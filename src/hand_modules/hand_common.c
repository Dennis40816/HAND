/*
 * Copyright (c) 2024 Dennis Liu, dennis48161025@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "hand_common.h"

#include "driver/usb_serial_jtag.h"

#ifndef HAND_DEFAULT_LOG_SERVER_IP
#define HAND_DEFAULT_LOG_SERVER_IP "192.168.1.17"
#endif

static const char* TAG = "HAND_COMMON";

// static hand_i2c_init() {}
// static hand_spi_init()
// {

// }
// static hand_gpio_init()
// {

// }
// static hand_intr_init()
// {

// }

esp_err_t hand_init(const char* ssid, const char* password)
{
  esp_err_t ret = ESP_OK;

  /* wait for system starts if usb plugged in */
  if (usb_serial_jtag_is_connected())
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  ret = hand_wifi_module_mount(NULL);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "hand_wifi_module_mount failed");
    return ret;
  }

  hand_wifi_t wifi_settings;

  ret = hand_wifi_module_set_handle(&wifi_settings, ssid, password);

  // OR

  // ret = hand_wifi_module_get_default_handle(&wifi_settings);

  /* Note: modify the ssid and password according to your config */
  // const char* new_ssid = "NEW_SSID";
  // memcpy(&wifi_settings.config.ssid, &new_ssid, sizeof(new_ssid));
  // const char* new_password = "NEW_PASSWORD";
  // memcpy(&wifi_settings.config.password, &new_password,
  // sizeof(new_password));

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "hand_wifi_module_get_default_handle failed");
    return ret;
  }

  ESP_LOGI(TAG, "Default Wi-Fi SSID: %s", (char*)&wifi_settings.config.ssid);
  ESP_LOGI(TAG, "Default Wi-Fi Password: %s",
           (char*)&wifi_settings.config.password);

  hand_wifi_module_update_handler(NULL);

  /* should connect */
  ret = hand_wifi_module_init(&wifi_settings, true);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "hand_wifi_module_init failed");
    return ret;
  }

  hand_terminal_t terminal_setting = {
      .local_server = {.server_type = HAND_UDP_SERVER,
                       .fcntl_flag = O_NONBLOCK,
                       .addr_family = HAND_AF_INET,
                       .addr = {.port = 12345}},
      .dest_addr = {.ip = HAND_DEFAULT_LOG_SERVER_IP, .port = 12345}};

  ret = hand_terminal_module_mount(NULL);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "hand_terminal_module_mount failed");
    return ret;
  }

  ret = hand_terminal_module_init(&terminal_setting);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "hand_terminal_module_init failed");
    return ret;
  }

  /* Init hand hardware begin */
  /* XXX: exclude CH101 related config */

  /* TODO: get all gpio states */
  // gpio_dump_all_io_configuration();

  return ret;
}