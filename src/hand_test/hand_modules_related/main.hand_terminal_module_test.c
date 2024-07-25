#include "hand_common.h"

#include "hand_wifi/hand_wifi_module.h"
#include "hand_terminal/hand_terminal_module.h"

static const char* TAG = "HAND_TERMINAL_MODULE_TEST";

void app_main()
{
  vTaskDelay(pdMS_TO_TICKS(3000));

  hand_wifi_module_mount(NULL);

  hand_wifi_t wifi_settings;

  hand_wifi_module_get_default_handle(&wifi_settings);

  /* Note: modify the ssid and password according to your config */
  // const char* new_ssid = "NEW_SSID";
  // memcpy(&wifi_settings.config.ssid, &new_ssid, sizeof(new_ssid));

  hand_wifi_module_update_handler(NULL);

  /* should connect */
  hand_wifi_module_init(&wifi_settings, true);

  hand_terminal_t terminal_setting = {
      .local_server = {.server_type = HAND_UDP_SERVER,
                       .fcntl_flag = O_NONBLOCK,
                       .addr_family = HAND_AF_INET,
                       .addr = {.port = 12345}},
      .dest_addr = {.ip = "192.168.1.17", .port = 12345}};

  hand_terminal_module_mount(NULL);
  hand_terminal_module_init(&terminal_setting);
}