#include "hand_common.h"
#include "hand_task/hand_task.h"
#include "hand_task/hand_task_priority.h"
#include "hand_server/hand_server.h"
#include "hand_global.h"

static const char* TAG = "HAND_DISTANCE_GROUP_TEST";

void app_main(void)
{
  /**
   * @brief Init the following
   *
   * - hand_global_device_handle
   * - hand_terminal
   *
   */
  hand_init(HAND_WIFI_MODULE_DEFAULT_SSID, HAND_WIFI_MODULE_DEFAULT_PASSWORD,
            true);

  /* create data tcp client socket */
  /* TODO: move to hand_init */
  hand_tcp_client_config_t tcp_client_config = {
      .addr_family = HAND_AF_INET,
      .client_name = "control_tcp_server",
      .server_addr = {.ip = HAND_DEFAULT_CONTROL_SERVER_IP,
                      .port = HAND_DEFAULT_CONTROL_SERVER_PORT}};

  int client_socket;
  esp_err_t err = hand_client_connect(&tcp_client_config, &client_socket);

  if (err == ESP_OK)
  {
    ESP_LOGI(TAG, "Client connected successfully, socket_fd: %d",
             client_socket);
    /* Create a task to send test data */
    xTaskCreate(hand_task_vl53l1x_send_data, "hand_task_vl53l1x_send_data",
                HAND_TASK_SS_VL53L1X_SEND_DATA, &client_socket,
                HAND_TASK_PRIORITY_VL53L1X_SEND_DATA,
                &hand_global_task_handle.vl53l1x_send_data_handle);
    xTaskCreate(hand_task_ch101_send_data, "hand_task_ch101_send_data",
                HAND_TASK_SS_CH101_SEND_DATA, &client_socket,
                HAND_TASK_PRIORITY_CH101_SEND_DATA,
                &hand_global_task_handle.ch101_send_data_handle);
  }
  else
  {
    ESP_LOGE(TAG, "Failed to connect to server, error: %d", err);
  }

  /* start up tasks */
  xTaskCreate(hand_task_vl53l1x_from_queue_to_ppb,
              "hand_task_vl53l1x_from_queue_to_ppb",
              HAND_TASK_SS_VL53L1X_FROM_QUEUE_TO_PPB, NULL,
              HAND_TASK_PRIORITY_VL53L1X_FROM_QUEUE_TO_PPB,
              &hand_global_task_handle.vl53l1x_from_queue_to_ppb_handle);

  xTaskCreate(hand_task_vl53l1x_collect_data, "hand_task_vl53l1x_collect_data",
              HAND_TASK_SS_VL53L1X_COLLECT_DATA, NULL,
              HAND_TASK_PRIORITY_VL53L1X_COLLECT_DATA,
              &hand_global_task_handle.vl53l1x_collect_data_handle);

  xTaskCreate(hand_task_ch101_from_queue_to_ppb,
              "hand_task_ch101_from_queue_to_ppb",
              HAND_TASK_SS_CH101_FROM_QUEUE_TO_PPB, NULL,
              HAND_TASK_PRIORITY_CH101_FROM_QUEUE_TO_PPB,
              &hand_global_task_handle.ch101_from_queue_to_ppb_handle);

  xTaskCreate(hand_task_ch101_collect_data, "hand_task_ch101_collect_data",
              HAND_TASK_SS_CH101_COLLECT_DATA, NULL,
              HAND_TASK_PRIORITY_CH101_COLLECT_DATA,
              &hand_global_task_handle.ch101_collect_data_handle);

  ESP_LOGI(TAG, "Create hand_task_alive");

  xTaskCreate(hand_task_alive, "hand_task_alive", HAND_TASK_SS_ALIVE, NULL,
              HAND_TASK_PRIORITY_ALIVE, &hand_global_task_handle.alive_handle);
}