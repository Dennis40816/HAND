#include "tcp_client.h"

static const char *TAG = "TCP_CLIENT";

// WiFi event group and bits
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

// WiFi event handler function
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
  {
    esp_wifi_connect();
  }
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
  {
    ESP_LOGI(TAG, "Disconnected from WiFi, reconnecting...");
    esp_wifi_connect();
  }
  else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

// Connect to WiFi using predefined SSID and password
void connect_to_wifi(void)
{
  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &wifi_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &wifi_event_handler, NULL));

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = TEST_HAND_WIFI_SSID,
              .password = TEST_HAND_WIFI_PASSWORD,
              .threshold.authmode = WIFI_AUTH_WPA2_PSK,
          },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", TEST_HAND_WIFI_SSID);

  // Wait for connection
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                         pdFALSE, pdTRUE, portMAX_DELAY);
  if (bits & WIFI_CONNECTED_BIT)
  {
    ESP_LOGI(TAG, "Connected to WiFi");
  }
  else
  {
    ESP_LOGE(TAG, "Failed to connect to WiFi");
  }
}

// Send data to the TCP Echo Server and receive response
void tcp_client_send(const char *data, char *recv_buf, int buf_size)
{
  int sock = 0;
  struct sockaddr_in server_addr;

  const char *server_ip = TEST_HAND_TCP_ECHO_SERVER_IP;

  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
  {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    return;
  }
  ESP_LOGI(TAG, "Socket created, connecting to %s:%d", server_ip, SERVER_PORT);

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(SERVER_PORT);
  server_addr.sin_addr.s_addr = inet_addr(server_ip);

  int err = connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
  if (err != 0)
  {
    ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
    close(sock);
    return;
  }
  ESP_LOGI(TAG, "Successfully connected");

  int to_write = strlen(data);
  int written = write(sock, data, to_write);
  if (written < 0)
  {
    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    close(sock);
    return;
  }
  ESP_LOGI(TAG, "Message sent");

  int len = read(sock, recv_buf, buf_size - 1);
  if (len < 0)
  {
    ESP_LOGE(TAG, "Receive failed: errno %d", errno);
  }
  else
  {
    recv_buf[len] = 0;
    ESP_LOGI(TAG, "Received %d bytes: %s", len, recv_buf);
  }

  close(sock);
}
