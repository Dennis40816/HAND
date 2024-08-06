#include "hand_common.h"
#include "hand_server/hand_server.h"
#include "hand_data/proto/hand_data.pb.h"
#include "pb_encode.h"
#include "esp_system.h"
#include "esp_timer.h"

static const char *TAG = "HAND_TCP_CLIENT_TEST";

#define NUM_SENSORS 2
#define SENSOR_DATA_COUNT 10
#define BUFFER_SIZE 2048
#define SERVER_IP "192.168.1.17"
#define SERVER_PORT 8055
#define TASK_DELAY_MS 1000

int64_t timestamps[SENSOR_DATA_COUNT];
HandDataMsg data_msgs[NUM_SENSORS];

/* Callback function to encode the sensor data */
bool encode_sensor_data(pb_ostream_t *stream, const pb_field_t *field,
                        void *const *arg)
{
  float *sensor_data = (float *)(*arg);
  return pb_encode_tag_for_field(stream, field) &&
         pb_encode_string(stream, (uint8_t *)sensor_data, SENSOR_DATA_COUNT * sizeof(float));
}

/* Callback function to encode the timestamps */
bool encode_timestamps(pb_ostream_t *stream, const pb_field_t *field,
                       void *const *arg)
{
  int64_t *timestamps = (int64_t *)(*arg);
  for (int i = 0; i < SENSOR_DATA_COUNT; i++)
  {
    if (!pb_encode_tag_for_field(stream, field))
    {
      return false;
    }
    if (!pb_encode_varint(stream, timestamps[i]))
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

/**
 * @brief Task to send VL53L1X simulated data over TCP.
 *
 * @param pvParameters Pointer to the client socket.
 */
static void send_task(void *pvParameters)
{
  int client_socket = *(int *)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  float sensor_data1[SENSOR_DATA_COUNT];
  float sensor_data2[SENSOR_DATA_COUNT];
  uint8_t buffer[BUFFER_SIZE];

  while (1)
  {
    HandMsg hand_msg = HandMsg_init_zero;

    // Set direction and message type
    hand_msg.direction = HandMsgDirection_FROM_HAND;
    hand_msg.msg_type = HandMainMsgType_DATA;
    hand_msg.chip_type = HandChipType_VL53L1X;

    // Set content
    hand_msg.which_content = HandMsg_data_wrapper_tag;

    // Generate random sensor data and timestamps
    for (int i = 0; i < SENSOR_DATA_COUNT; i++)
    {
      sensor_data1[i] = (float)esp_random() / UINT32_MAX * 130.0f;
      sensor_data2[i] = (float)esp_random() / UINT32_MAX * 130.0f;
      timestamps[i] = esp_timer_get_time();  // Example timestamps
    }

    // Fill first sensor data
    HandDataMsg data_msg1 = HandDataMsg_init_zero;
    data_msg1.source = HandChipInstance_VL53L1X_SENSOR1;
    data_msg1.data_type = HandDataType_FLOAT;
    data_msg1.data_count = SENSOR_DATA_COUNT;
    data_msg1.has_timestamp = false;
    data_msg1.timestamps.funcs.encode = encode_timestamps;
    data_msg1.timestamps.arg = timestamps;
    data_msg1.data.funcs.encode = encode_sensor_data;
    data_msg1.data.arg = sensor_data1;

    // Fill second sensor data
    HandDataMsg data_msg2 = HandDataMsg_init_zero;
    data_msg2.source = HandChipInstance_VL53L1X_SENSOR2;
    data_msg2.data_type = HandDataType_FLOAT;
    data_msg2.data_count = SENSOR_DATA_COUNT;
    data_msg2.has_timestamp = false;
    data_msg2.timestamps.funcs.encode = encode_timestamps;
    data_msg2.timestamps.arg = timestamps;
    data_msg2.data.funcs.encode = encode_sensor_data;
    data_msg2.data.arg = sensor_data2;

    data_msgs[0] = data_msg1;
    data_msgs[1] = data_msg2;

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

    // Delay until the next cycle
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_DELAY_MS));
  }
}

/**
 * @brief Main application entry point.
 */
void app_main(void)
{
  hand_init();

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
                tskIDLE_PRIORITY + 1, NULL);
  }
  else
  {
    ESP_LOGE(TAG, "Failed to connect to server, error: %d", err);
  }
}
