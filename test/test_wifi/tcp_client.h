#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "string.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

// Macro definitions to ensure they are available for use
// defined in `hand_test_hook.py`
#ifndef TEST_HAND_TCP_ECHO_SERVER_IP
#error \
    "TEST_HAND_TCP_ECHO_SERVER_IP not defined. Please define it in platformio.ini or through the build script."
#endif

#ifndef TEST_HAND_WIFI_SSID
#error \
    "TEST_HAND_WIFI_SSID not defined. Please define it in platformio.ini or through the build script."
#endif

#ifndef TEST_HAND_WIFI_PASSWORD
#error \
    "TEST_HAND_WIFI_PASSWORD not defined. Please define it in platformio.ini or through the build script."
#endif

#define SERVER_PORT 6020
#define BUFFER_SIZE 128

// Function declarations
void connect_to_wifi(void);
void tcp_client_send(const char *data, char *recv_buf, int buf_size);

#endif  // TCP_CLIENT_H
