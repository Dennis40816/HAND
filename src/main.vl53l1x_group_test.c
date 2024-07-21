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

#include "vl53l1x.h"

/* default config */
#define VL53L1X_DEFAULT_USER_TIMING_BUDGET_MS  (50)
#define VL53L1X_DEFAULT_USER_MEASURE_PERIOD_MS (100)

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

static const char* TAG = "VL53L1X_GROUP_TEST";

/* create vl53l1x devices */
VL53L1_Dev_t vl53l1x_1 = {.I2cHandle = I2C_NUM_0,
                          .comms_speed_khz = (uint16_t)400,
                          .I2cDevAddr = VL53L1X_DEFAULT_I2C_ADDRESS,
                          .new_data_ready_poll_duration_ms = 100};

VL53L1_Dev_t vl53l1x_2 = {.I2cHandle = I2C_NUM_0,
                          .comms_speed_khz = (uint16_t)400,
                          .I2cDevAddr = VL53L1X_DEFAULT_I2C_ADDRESS,
                          .new_data_ready_poll_duration_ms = 100};

static void IRAM_ATTR gpio_isr_handler(void* arg)
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

static void collect_data(void* arg)
{
  while (1)
  {
    EventBits_t uxBits =
        xEventGroupWaitBits(vl53l1x_data_ready_event_group,
                            VL53L1X_1_DATA_READY_BIT | VL53L1X_2_DATA_READY_BIT,
                            pdTRUE,   // clear wait bit when exit
                            pdFALSE,  // fire once any bit is set
                            portMAX_DELAY);

    if (uxBits & VL53L1X_1_DATA_READY_BIT)
    {
      VL53L1_RangingMeasurementData_t RangingData;
      VL53L1_Error status =
          VL53L1_GetRangingMeasurementData(&vl53l1x_1, &RangingData);
      if (status == 0)
      {
        ESP_LOGI(TAG, "VL53L1X_1: %3.1f (cm)",
                 RangingData.RangeMilliMeter / 10.0);
      }
      status = VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x_1);
      if (status != VL53L1_ERROR_NONE)
      {
        ESP_LOGW(
            TAG,
            "Status of `VL53L1_ClearInterruptAndStartMeasurement` is: {%d}",
            status);
      }
    }

    if (uxBits & VL53L1X_2_DATA_READY_BIT)
    {
      VL53L1_RangingMeasurementData_t RangingData;
      VL53L1_Error status =
          VL53L1_GetRangingMeasurementData(&vl53l1x_2, &RangingData);
      if (status == 0)
      {
        ESP_LOGI(TAG, "VL53L1X_2: %3.1f (cm)",
                 RangingData.RangeMilliMeter / 10.0);
      }
      status = VL53L1_ClearInterruptAndStartMeasurement(&vl53l1x_2);
      if (status != VL53L1_ERROR_NONE)
      {
        ESP_LOGW(
            TAG,
            "Status of `VL53L1_ClearInterruptAndStartMeasurement` is: {%d}",
            status);
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
  VL53L1X_SetI2CAddress(&vl53l1x_1, VL53L1X_1_NEW_I2C_ADDRESS);

  /* change VL53L1X_2 i2c address to 0x62 */
  VL53L1_GpioXshutdown(VL53L1X_2_INDEX, VL53L1X_XSHUT_HIGH);
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
                       (void*)VL53L1X_1_GPIO0_PIN);

  /* add interrupt service for VL53L1X_2 */
  gpio_isr_handler_add(VL53L1X_2_GPIO0_PIN, gpio_isr_handler,
                       (void*)VL53L1X_2_GPIO0_PIN);
  /* debug only */
  xTaskCreate(collect_data, "collect_data", 4096, NULL, 10, NULL);
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

void app_main(void)
{
  esp_log_level_set("*", ESP_LOG_DEBUG);
  vTaskDelay(pdMS_TO_TICKS(3000));

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
}