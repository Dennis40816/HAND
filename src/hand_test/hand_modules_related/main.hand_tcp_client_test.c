#include "hand_common.h"
#include "hand_server/hand_server.h"

static const char* TAG = "HAND_TCP_CLIENT_TEST";

static void send_task(void* pvParameters)
{
  int client_socket = *(int*)pvParameters;
  const char* test_str = "HAND_TEST_TCP_CLIENT";

  while (1)
  {
    int ret = send(client_socket, test_str, strlen(test_str), 0);
    if (ret < 0)
    {
      ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }
    else
    {
      ESP_LOGI(TAG, "Message sent successfully");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void app_main(void)
{
  hand_init();

  /* Create a tcp client */
  hand_tcp_client_config_t tcp_client_config = {
      .addr_family = HAND_AF_INET,
      .client_name = "test_tcp_client",
      .server_addr = {.ip = "192.168.1.17", .port = 8055}};

  int client_socket;
  esp_err_t err = hand_client_connect(&tcp_client_config, &client_socket);

  if (err == ESP_OK)
  {
    ESP_LOGI(TAG, "Client connected successfully, socket_fd: %d",
             client_socket);
    /* using the socket to send test string */
    xTaskCreate(send_task, "send_task", 4096, &client_socket,
                tskIDLE_PRIORITY + 1, NULL);
  }
  else
  {
    ESP_LOGE(TAG, "Failed to connect to server, error: %d", err);
  }
}