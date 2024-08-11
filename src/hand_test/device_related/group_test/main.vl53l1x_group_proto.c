/**
 * Good Reference:
 * 1. How ROI set works: we set upper left and lower right corner coordinate
 * (ranging from (0,0) to (15, 15))
 * https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library/issues/9#issuecomment-419504574
 *
 * 2. Port to ESPIDF
 * https://blog.csdn.net/qq_20515461/article/details/99293706
 *
 * 3. https://blog.csdn.net/tiramisu_L/article/details/90729964
 * 4. API:
 * https://www.st.com/resource/en/user_manual/um2356-vl53l1x-api-user-manual-stmicroelectronics.pdf
 */

#include "hand_common.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "vl53l1x.h"

/* default config */
#define BASE_TASK_PRIORITY (10)

#define VL53L1X_DEFAULT_USER_TIMING_BUDGET_MS  (50)
#define VL53L1X_DEFAULT_USER_MEASURE_PERIOD_MS (100)

#define BYTE_NUM_PLACEHOLDER 1
#define NUM_SENSORS          2
#define SENSOR_DATA_COUNT    100
#define PB_BUFFER_SIZE       2048

#ifndef HAND_DEFAULT_LOG_SERVER_IP
#define SERVER_IP            "192.168.0.170"
#else
#define SERVER_IP HAND_DEFAULT_LOG_SERVER_IP
#endif

#define SERVER_PORT          8055
#define TASK_DELAY_MS        500

/* user config */
// This must be uint16_t [20, 1000]
#define VL53L1X_USER_TIMING_BUDGET_MS (VL53L1X_DEFAULT_USER_TIMING_BUDGET_MS)
// This argument should be equal or larger than (VL53L1X_USER_TIMING_BUDGET_MS +
// 4)
#define VL53L1X_USER_MEASURE_PERIOD_MS (VL53L1X_DEFAULT_USER_MEASURE_PERIOD_MS)

/* macros */
#define VL53L1X_1_INDEX     (1)
#define VL53L1X_2_INDEX     (2)
#define VL53L1X_XSHUT_HIGH  (1)
#define VL53L1X_XSHUT_LOW   (0)
#define VL53L1X_1_GPIO0_PIN (GPIO_NUM_17)
#define VL53L1X_2_GPIO0_PIN (GPIO_NUM_18)

#define VL53L1X_1_NEW_I2C_ADDRESS   (0x60)
#define VL53L1X_2_NEW_I2C_ADDRESS   (0x62)
#define VL53L1X_DEFAULT_I2C_ADDRESS (0x52)

#define VL53L1X_MODEL_ID_INDEX      0x010F
#define VL53L1X_MODULE_TYPE_INDEX   0x0110
#define VL53L1X_MASK_REVISION_INDEX 0x0111

/* data ready interrupt */
static EventGroupHandle_t vl53l1x_data_ready_event_group;
#define VL53L1X_1_DATA_READY_BIT (1 << 0)
#define VL53L1X_2_DATA_READY_BIT (1 << 1)

#define PING_PONG_BUFFER_SIZE 128

typedef struct hand_vl53l1x_data_t
{
  int64_t timestamps[PING_PONG_BUFFER_SIZE];
  float sensor1_data[PING_PONG_BUFFER_SIZE];
  float sensor2_data[PING_PONG_BUFFER_SIZE];
} hand_vl53l1x_data_t;

typedef struct PingPongBuffer_t
{
  hand_vl53l1x_data_t data[2];
  uint8_t current_index;
  bool ping_pong_flag;
} PingPongBuffer_t;

static PingPongBuffer_t ping_pong_buffer = {0};
static QueueHandle_t data_queue;
static SemaphoreHandle_t ping_pong_buffer_mutex;
static SemaphoreHandle_t first_data_in_queue;

/* proto related defines */
#include "hand_server/hand_server.h"
#include "hand_data/proto/hand_data.pb.h"
#include "pb_encode.h"
#include "esp_system.h"
#include "esp_timer.h"

HandDataMsg data_msgs[NUM_SENSORS];

typedef struct vl53l1x_data_arg_t
{
  float *fp;
  int count;
} vl53l1x_data_arg_t;

typedef struct timestamps_data_arg_t
{
  int64_t *p;
  int count;
} timestamps_data_arg_t;

typedef struct SensorData_t
{
  int64_t timestamp;
  float sensor1_data;
  float sensor2_data;
} SensorData_t;

/* Callback function to encode the sensor data */
bool encode_sensor_data(pb_ostream_t *stream, const pb_field_t *field,
                        void *const *arg)
{
  vl53l1x_data_arg_t *data_arg = (vl53l1x_data_arg_t *)(*arg);
  return pb_encode_tag_for_field(stream, field) &&
         pb_encode_string(stream, (uint8_t *)data_arg->fp,
                          data_arg->count * sizeof(float));
}

/* Callback function to encode the timestamps */
bool encode_timestamps(pb_ostream_t *stream, const pb_field_t *field,
                       void *const *arg)
{
  timestamps_data_arg_t *t_arg = (timestamps_data_arg_t *)(*arg);
  for (int i = 0; i < t_arg->count; i++)
  {
    if (!pb_encode_tag_for_field(stream, field))
    {
      return false;
    }
    if (!pb_encode_varint(stream, (t_arg->p)[i]))
    {
      return false;
    }
  }

  return true;
}

/* Callback function to encode the data messages array */
bool encode_data_msgs(pb_ostream_t *stream, const pb_field_t *field,
                      void *const *arg)
{
  HandDataMsg *data_msgs = (HandDataMsg *)(*arg);
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    if (!pb_encode_tag_for_field(stream, field)) return false;
    if (!pb_encode_submessage(stream, HandDataMsg_fields, &data_msgs[i]))
      return false;
  }
  return true;
}

static bool write_bytes_count_to_buf(uint8_t *buf, size_t offset,
                                     uint32_t value)
{
  pb_ostream_t stream = pb_ostream_from_buffer(buf + offset, sizeof(uint32_t));
  return pb_encode_fixed32(&stream, &value);
}

static const char *TAG = "VL53L1X_GROUP_PROTO_TEST";

/* create vl53l1x devices */
VL53L1_Dev_t vl53l1x_1 = {.I2cHandle = I2C_NUM_0,
                          .comms_speed_khz = (uint16_t)400,
                          .I2cDevAddr = VL53L1X_DEFAULT_I2C_ADDRESS,
                          .new_data_ready_poll_duration_ms = 100};

VL53L1_Dev_t vl53l1x_2 = {.I2cHandle = I2C_NUM_0,
                          .comms_speed_khz = (uint16_t)400,
                          .I2cDevAddr = VL53L1X_DEFAULT_I2C_ADDRESS,
                          .new_data_ready_poll_duration_ms = 100};

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
  int pin_num = (int)arg;

  if (pin_num == VL53L1X_1_GPIO0_PIN)
  {
    xEventGroupSetBitsFromISR(vl53l1x_data_ready_event_group,
                              VL53L1X_1_DATA_READY_BIT, NULL);
  }
  else if (pin_num == VL53L1X_2_GPIO0_PIN)
  {
    xEventGroupSetBitsFromISR(vl53l1x_data_ready_event_group,
                              VL53L1X_2_DATA_READY_BIT, NULL);
  }
}

static void collect_data(void *arg)
{
  while (1)
  {
    EventBits_t uxBits =
        xEventGroupWaitBits(vl53l1x_data_ready_event_group,
                            VL53L1X_1_DATA_READY_BIT | VL53L1X_2_DATA_READY_BIT,
                            pdTRUE,  // clear wait bit when exit
                            pdTRUE,  // fire at all bits are set
                            portMAX_DELAY);

    int64_t timestamp = esp_timer_get_time();  // Get the current timestamp

    SensorData_t sensor_data = {0};
    sensor_data.timestamp = timestamp;

    if (uxBits & VL53L1X_1_DATA_READY_BIT)
    {
      VL53L1_RangingMeasurementData_t RangingData;
      VL53L1_Error status =
          VL53L1_GetRangingMeasurementData(&vl53l1x_1, &RangingData);
      if (status == 0)
      {
        // ESP_LOGI(TAG, "VL53L1X_1: %3.1f (cm)",
        //          RangingData.RangeMilliMeter / 10.0);
        sensor_data.sensor1_data = RangingData.RangeMilliMeter / 10.0;
      }
      status = VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x_1);
      if (status != VL53L1_ERROR_NONE)
      {
        ESP_LOGW(
            TAG,
            "Status of `VL53L1_ClearInterruptAndStartMeasurement` is: {%d}",
            status);
        /* TODO: need to handle this */
      }
    }

    if (uxBits & VL53L1X_2_DATA_READY_BIT)
    {
      VL53L1_RangingMeasurementData_t RangingData;
      VL53L1_Error status =
          VL53L1_GetRangingMeasurementData(&vl53l1x_2, &RangingData);
      if (status == 0)
      {
        // ESP_LOGI(TAG, "VL53L1X_2: %3.1f (cm)",
        //          RangingData.RangeMilliMeter / 10.0);
        sensor_data.sensor2_data = RangingData.RangeMilliMeter / 10.0;
      }
      status = VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x_2);
      if (status != VL53L1_ERROR_NONE)
      {
        ESP_LOGW(
            TAG,
            "Status of `VL53L1_ClearInterruptAndStartMeasurement` is: {%d}",
            status);
        /* TODO: need to handle this */
      }
    }

    // Send collected data to the store data task
    xQueueSend(data_queue, &sensor_data, portMAX_DELAY);
  }
}

/* Task to store data into ping-pong buffer */
static void store_data(void *arg)
{
  static bool first_data_in = false;
  SensorData_t sensor_data;
  while (1)
  {
    // Receive data from the queue
    if (xQueueReceive(data_queue, &sensor_data, portMAX_DELAY) == pdTRUE)
    {
      // Critical section: move collected data to ping-pong buffer
      xSemaphoreTake(ping_pong_buffer_mutex, portMAX_DELAY);  // Take the mutex

      uint8_t buffer_index = ping_pong_buffer.ping_pong_flag ? 1 : 0;
      ping_pong_buffer.data[buffer_index]
          .timestamps[ping_pong_buffer.current_index] = sensor_data.timestamp;
      ping_pong_buffer.data[buffer_index]
          .sensor1_data[ping_pong_buffer.current_index] =
          sensor_data.sensor1_data;
      ping_pong_buffer.data[buffer_index]
          .sensor2_data[ping_pong_buffer.current_index] =
          sensor_data.sensor2_data;

      ping_pong_buffer.current_index++;
      if (ping_pong_buffer.current_index >= PING_PONG_BUFFER_SIZE)
      {
        ping_pong_buffer.current_index = 0;
        ping_pong_buffer.ping_pong_flag = !ping_pong_buffer.ping_pong_flag;
      }

      xSemaphoreGive(ping_pong_buffer_mutex);  // Give the mutex

      if(first_data_in == false)
      {
          xSemaphoreGive(first_data_in_queue);
          first_data_in = true;
      }
    }
  }
}

static void i2c_bus_init()
{
  const i2c_port_t i2c0_port = I2C_NUM_0;
  i2c_config_t i2c0_config = {.mode = I2C_MODE_MASTER,
                              .sda_io_num = GPIO_NUM_1,
                              .scl_io_num = GPIO_NUM_0,
                              .sda_pullup_en = GPIO_PULLUP_DISABLE,
                              .scl_pullup_en = GPIO_PULLUP_DISABLE,
                              .master.clk_speed = 400000};

  ESP_ERROR_CHECK(i2c_param_config(i2c0_port, &i2c0_config));

  /* WARNING: we disable the i2c master interrupt (which samg55 enable) */
  /* See i2c_master_register_event_callbacks() for more information */
  ESP_ERROR_CHECK(i2c_driver_install(i2c0_port, i2c0_config.mode, 0, 0, 0));
}

/* remain xshut high after call this function */
static void vl53l1x_change_i2c_address()
{
  VL53L1_GpioXshutdown(VL53L1X_1_INDEX, VL53L1X_XSHUT_LOW);
  VL53L1_GpioXshutdown(VL53L1X_2_INDEX, VL53L1X_XSHUT_LOW);

  /* change VL53L1X_1 i2c address to 0x60 */
  VL53L1_GpioXshutdown(VL53L1X_1_INDEX, VL53L1X_XSHUT_HIGH);
  VL53L1_WaitDeviceBooted(&vl53l1x_1);
  VL53L1X_SetI2CAddress(&vl53l1x_1, VL53L1X_1_NEW_I2C_ADDRESS);

  /* change VL53L1X_2 i2c address to 0x62 */

  VL53L1_GpioXshutdown(VL53L1X_2_INDEX, VL53L1X_XSHUT_HIGH);
  VL53L1_WaitDeviceBooted(&vl53l1x_2);
  VL53L1X_SetI2CAddress(&vl53l1x_2, VL53L1X_2_NEW_I2C_ADDRESS);
}

static void add_gpio0_interrupt()
{
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.pin_bit_mask =
      (1ULL << VL53L1X_1_GPIO0_PIN) | (1ULL << VL53L1X_2_GPIO0_PIN);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);

  /* init eventgroup */
  vl53l1x_data_ready_event_group = xEventGroupCreate();

  /* add interrupt service for VL53L1X_1 */
  gpio_install_isr_service(0);
  gpio_isr_handler_add(VL53L1X_1_GPIO0_PIN, gpio_isr_handler,
                       (void *)VL53L1X_1_GPIO0_PIN);

  /* add interrupt service for VL53L1X_2 */
  gpio_isr_handler_add(VL53L1X_2_GPIO0_PIN, gpio_isr_handler,
                       (void *)VL53L1X_2_GPIO0_PIN);
  /* debug only */
  xTaskCreate(collect_data, "collect_data", 4096, NULL, BASE_TASK_PRIORITY,
              NULL);
  xTaskCreate(store_data, "store_data", 4096, NULL, BASE_TASK_PRIORITY, NULL);
}

static void vl53l1x_init(VL53L1_DEV dev, bool calibration_en)
{
  /* ROI 4*4 middle*/
  VL53L1_UserRoi_t default_roi = {
      .TopLeftX = 6, .TopLeftY = 9, .BotRightX = 9, .BotRightY = 6};

  ESP_LOGW(TAG, "Init process starts, all return status should be: {%d}",
           VL53L1_ERROR_NONE);

  /* init */
  int status = VL53L1_WaitDeviceBooted(dev);
  ESP_LOGI(TAG, "Status of `VL53L1_WaitDeviceBooted` is: {%d}", status);
  status = VL53L1_DataInit(dev);
  ESP_LOGI(TAG, "Status of `VL53L1_DataInit` is: {%d}", status);
  status = VL53L1_StaticInit(dev);
  ESP_LOGI(TAG, "Status of `VL53L1_StaticInit` is: {%d}", status);

  /* set perset mode */
  status = VL53L1_SetPresetMode(dev, VL53L1_PRESETMODE_AUTONOMOUS);
  ESP_LOGI(TAG, "Status of `VL53L1_SetPresetMode` is: {%d}", status);

  /* run calibration if enable */
  if (calibration_en)
  {
    /* TODO: run calibration */
  }

  status = VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_SHORT);
  ESP_LOGI(TAG, "Status of `VL53L1_SetDistanceMode` is: {%d}", status);
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(
      dev, VL53L1X_USER_TIMING_BUDGET_MS * 1000);
  ESP_LOGI(TAG,
           "Status of `VL53L1_SetMeasurementTimingBudgetMicroSeconds` is: {%d}",
           status);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(
      dev, VL53L1X_USER_MEASURE_PERIOD_MS);
  ESP_LOGI(TAG,
           "Status of `VL53L1_SetInterMeasurementPeriodMilliSeconds` is: {%d}",
           status);

  status = VL53L1_SetUserROI(dev, &default_roi);
  ESP_LOGI(TAG, "Status of `VL53L1_SetUserROI` is: {%d}", status);

  ESP_LOGW(TAG, "Init process end");
}

static void send_task(void *pvParameters)
{
  int client_socket = *(int *)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t buffer[PB_BUFFER_SIZE];
  int send_buffer_index = 0;

  xSemaphoreTake(first_data_in_queue, portMAX_DELAY);

  while (1)
  {
    HandMsg hand_msg = HandMsg_init_zero;

    // Set direction and message type
    hand_msg.bytes_count = BYTE_NUM_PLACEHOLDER;  // just a place holder
    hand_msg.direction = HandMsgDirection_FROM_HAND;
    hand_msg.msg_type = HandMainMsgType_DATA;
    hand_msg.chip_type = HandChipType_VL53L1X;

    // Set content
    hand_msg.which_content = HandMsg_data_wrapper_tag;

    // Get the current data index
    uint8_t send_data_index = 0;

    // Take the mutex before accessing shared resources, may blocked here
    if (xSemaphoreTake(ping_pong_buffer_mutex, portMAX_DELAY) == pdTRUE)
    {
      // Determine which buffer to send
      send_buffer_index = ping_pong_buffer.ping_pong_flag ? 1 : 0;
      send_data_index = ping_pong_buffer.current_index;
      // For to swap buffers
      ping_pong_buffer.current_index = 0;
      ping_pong_buffer.ping_pong_flag = !ping_pong_buffer.ping_pong_flag;

      // Release the mutex immediately after accessing shared data
      xSemaphoreGive(ping_pong_buffer_mutex);
    }

    if (send_data_index != 0)
    {
      ESP_LOGI(TAG, "send data index is %d", send_data_index);
      /* format args */
      timestamps_data_arg_t timestamps_arg = {
          .p = ping_pong_buffer.data[send_buffer_index].timestamps,
          .count = send_data_index  // data number
      };

      vl53l1x_data_arg_t vl53l1x_sensor1_data_arg = {
          .fp = ping_pong_buffer.data[send_buffer_index].sensor1_data,
          .count = send_data_index  // data number
      };

      vl53l1x_data_arg_t vl53l1x_sensor2_data_arg = {
          .fp = ping_pong_buffer.data[send_buffer_index].sensor2_data,
          .count = send_data_index  // data number
      };

      // Fill first sensor data
      HandDataMsg data_msg1 = HandDataMsg_init_zero;
      data_msg1.source = HandChipInstance_VL53L1X_SENSOR1;
      data_msg1.data_type = HandDataType_FLOAT;
      data_msg1.data_count = send_data_index;
      // use timestamps instead of timestamp
      data_msg1.has_timestamp = false;
      data_msg1.timestamps.funcs.encode = encode_timestamps;
      data_msg1.timestamps.arg = &timestamps_arg;
      data_msg1.data.funcs.encode = encode_sensor_data;
      data_msg1.data.arg = &vl53l1x_sensor1_data_arg;

      // Fill second sensor data
      HandDataMsg data_msg2 = HandDataMsg_init_zero;
      data_msg2.source = HandChipInstance_VL53L1X_SENSOR2;
      data_msg2.data_type = HandDataType_FLOAT;
      data_msg2.data_count = send_data_index;
      data_msg2.has_timestamp = false;
      data_msg2.timestamps.funcs.encode = encode_timestamps;
      data_msg2.timestamps.arg = &timestamps_arg;
      data_msg2.data.funcs.encode = encode_sensor_data;
      data_msg2.data.arg = &vl53l1x_sensor2_data_arg;

      HandDataMsg data_msgs[2] = {data_msg1, data_msg2};

      hand_msg.content.data_wrapper.data_msgs.funcs.encode = encode_data_msgs;
      hand_msg.content.data_wrapper.data_msgs.arg = data_msgs;

      size_t buffer_size = sizeof(buffer);

      // Timing start - serialization
      int64_t start_time = esp_timer_get_time();

      pb_ostream_t stream = pb_ostream_from_buffer(buffer, buffer_size);
      if (!pb_encode(&stream, HandMsg_fields, &hand_msg))
      {
        ESP_LOGE(TAG, "Encoding failed: %s", PB_GET_ERROR(&stream));
        continue;
      }

      int64_t end_time = esp_timer_get_time();
      int64_t encode_duration = end_time - start_time;
      ESP_LOGI(TAG,
               "Message encoded successfully, size: %zu bytes, time: %lld us",
               stream.bytes_written, encode_duration);

      // Overwrite buffer[1:4] by the function
      const size_t field1_offset = 1;
      write_bytes_count_to_buf(buffer, field1_offset, stream.bytes_written);

      // Timing start - sending data
      start_time = esp_timer_get_time();

      int ret = send(client_socket, buffer, stream.bytes_written, 0);
      if (ret < 0)
      {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
      }
      else
      {
        ESP_LOGI(TAG, "Message sent successfully");
      }

      end_time = esp_timer_get_time();
      int64_t transmit_duration = end_time - start_time;
      ESP_LOGI(TAG, "Data transmitted successfully, time: %lld us",
               transmit_duration);
    }
    // Delay until the next cycle
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_DELAY_MS));
  }
}

// You can overwrite these macros

// #define HAND_WIFI_MODULE_DEFAULT_SSID "YOUR_SSID"
// #define HAND_WIFI_MODULE_DEFAULT_PASSWORD "YOUR_SSID"

void app_main(void)
{
  hand_init(HAND_WIFI_MODULE_DEFAULT_SSID, HAND_WIFI_MODULE_DEFAULT_PASSWORD);

  // Initialize mutex
  first_data_in_queue = xSemaphoreCreateBinary();
  ping_pong_buffer_mutex = xSemaphoreCreateMutex();
  if (ping_pong_buffer_mutex == NULL)
  {
    ESP_LOGE(TAG, "Failed to create mutex for ping-pong buffer");
    return;
  }

  // Initialize data queue
  data_queue = xQueueCreate(10, sizeof(SensorData_t));
  if (data_queue == NULL)
  {
    ESP_LOGE(TAG, "Failed to create data queue");
    return;
  }

  /* XXX: Do not call "*", it will cause Wi-Fi crash */
  // D (123100) esp_netif_lwip: esp_netif_ip_lost_timer esp_netif:0x3fcab3f8
  // D (123100) esp_netif_lwip: if0x3fcab3f8 ip lost tmr: no need raise ip lost

  // event esp_log_level_set("*", ESP_LOG_DEBUG);

  i2c_bus_init();

  VL53L1_ConfigTca6408a(I2C_NUM_0);

  /* change i2c address */
  vl53l1x_change_i2c_address();

  /* try to read vl53l1x_1 */
  uint8_t model_id = 0x00;
  VL53L1_RdByte(&vl53l1x_1, VL53L1X_MODEL_ID_INDEX, &model_id);

  ESP_LOGI(TAG, "Reading from address: {0x%02X}", vl53l1x_1.I2cDevAddr);
  ESP_LOGI(TAG, "Model ID is: {0x%02X}", model_id);

  /* try to read vl53l1x_2 */
  model_id = 0x00;

  VL53L1_RdByte(&vl53l1x_2, VL53L1X_MODEL_ID_INDEX, &model_id);

  ESP_LOGI(TAG, "Reading from address: {0x%02X}", vl53l1x_2.I2cDevAddr);
  ESP_LOGI(TAG, "Model ID is: {0x%02X}", model_id);

  /* init both vl53l1x */
  vl53l1x_init(&vl53l1x_1, 0);
  vl53l1x_init(&vl53l1x_2, 0);

  /* add interrupt function */
  add_gpio0_interrupt();

  /* start measurement */
  VL53L1_Error ret;
  ret = VL53L1_StartMeasurement(&vl53l1x_1);
  ESP_LOGI(TAG, "Status of `VL53L1_StartMeasurement` is: {%d}", ret);
  ret = VL53L1_StartMeasurement(&vl53l1x_2);
  ESP_LOGI(TAG, "Status of `VL53L1_StartMeasurement` is: {%d}", ret);

  /* Create a TCP client */
  hand_tcp_client_config_t tcp_client_config = {
      .addr_family = HAND_AF_INET,
      .client_name = "test_tcp_client",
      .server_addr = {.ip = SERVER_IP, .port = SERVER_PORT}};

  int client_socket;
  esp_err_t err = hand_client_connect(&tcp_client_config, &client_socket);

  if (err == ESP_OK)
  {
    ESP_LOGI(TAG, "Client connected successfully, socket_fd: %d",
             client_socket);
    /* Create a task to send test data */
    xTaskCreate(send_task, "send_task", 8192, &client_socket,
                BASE_TASK_PRIORITY + 1, NULL);
  }
  else
  {
    ESP_LOGE(TAG, "Failed to connect to server, error: %d", err);
  }
}