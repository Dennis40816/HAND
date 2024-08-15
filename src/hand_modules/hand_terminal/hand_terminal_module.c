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

/* Includes */
#include "hand_terminal_module.h"
#include "hand_task/hand_task.h"
#include "hand_task/hand_task_priority.h"

#include "esp_netif.h"
#include "esp_log.h"

#include "driver/usb_serial_jtag.h"

/* Private variables */
static const char* TAG = "HAND_TERMINAL_MODULE";

static int _udp_socket;
/* INET version */
static struct sockaddr_in _dest_addr = {0};
static TaskHandle_t _recv_task_handle = NULL;

/* record for original vprintf */
static vprintf_like_t _original_vprintf = NULL;

/* Private functions */
static esp_err_t hand_terminal_module_create_server(
    hand_server_config_t* local_config, hand_server_address_t* dest_addr)
{
  if (local_config->addr_family == HAND_AF_INET6)
  {
    /* TODO: impl */
    ESP_LOGE(TAG,
             "Currently, there's no implementation for INET6 address! Abort");
    return ESP_ERR_NOT_SUPPORTED;
  }

  /* configure destination address for INET */
  if (local_config->addr_family == HAND_AF_INET)
  {
    _dest_addr.sin_addr.s_addr = inet_addr(dest_addr->ip);
    /* TODO: check to use `local_config->addr_family` or not */
    _dest_addr.sin_family = local_config->addr_family;
    _dest_addr.sin_port = htons(dest_addr->port);
  }

  /* check socket is invalid */
  if (_udp_socket > 0)
  {
    ESP_LOGW(TAG, "UDP socket is valid, going to close the socket. Fd: {%d}",
             _udp_socket);
    close(_udp_socket);
  }

  /* init default socket for server */
  if (local_config->server_type == HAND_UDP_SERVER)
  {
    /* SOCK_DGRAM for UDP server */
    int tmp_socket = socket(local_config->addr_family, SOCK_DGRAM, IPPROTO_UDP);
    if (tmp_socket < 0)
    {
      ESP_LOGE(TAG, "Create socket failed, socket value is: {%d}", tmp_socket);
      return ESP_ERR_INVALID_ARG;
    }

    _udp_socket = tmp_socket;
  }
  else
  {
    /* TODO: impl */
    ESP_LOGE(TAG,
             "Currently, there's no implementation for other type of server "
             "instead of UDP");
    return ESP_ERR_NOT_SUPPORTED;
  }

  /* bind to current ip and given port */
  struct sockaddr_in local_addr;
  local_addr.sin_family = local_config->addr_family;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = htons(local_config->addr.port);
  if (bind(_udp_socket, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0)
  {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    close(_udp_socket);
    return ESP_ERR_INVALID_ARG;
  }

  /* configure socket user flag */
  fcntl(_udp_socket, F_SETFL, local_config->fcntl_flag);

  /* get current server ip */
  esp_netif_ip_info_t ip_info;
  esp_err_t ret = hand_server_get_current_ip(&ip_info);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Got current ip failed due to error code: {%d}", ret);
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG,
           "Create server successfully. Already bind to ip: { " IPSTR
           " }, port: %d",
           IP2STR(&ip_info.ip), local_config->addr.port);

  return ESP_OK;
}

static esp_err_t hand_terminal_module_destroy_server(
    hand_server_config_t* local_config)
{
  // Check server type
  if (local_config->server_type == HAND_UDP_SERVER)
  {
    if (_udp_socket >= 0)
    {
      // Close the socket
      close(_udp_socket);
      _udp_socket = -1;  // Reset the socket handle
    }
    else
    {
      ESP_LOGW(TAG, "UDP socket was not initialized or already closed.");
      return ESP_ERR_INVALID_STATE;
    }
  }
  else
  {
    // Currently only UDP server is supported
    ESP_LOGE(TAG, "Currently, only UDP server is supported.");
    return ESP_ERR_NOT_SUPPORTED;
  }

  ESP_LOGI(TAG, "Server destroyed successfully.");
  return ESP_OK;
}

static int hand_esp_log_handler_vprintf(const char* format, va_list args)
{
  char log_buffer[512];
  int len = vsnprintf(log_buffer, sizeof(log_buffer), format, args);
  if (len > 0)
  {
    sendto(_udp_socket, log_buffer, len, 0, (struct sockaddr*)&_dest_addr,
           sizeof(_dest_addr));
  }

  // See: https://github.com/espressif/esp-idf/issues/9366
  if (usb_serial_jtag_is_connected())
  {
    return vprintf(format, args);
  }
  else
  {
    return len;
  }
}

static void hand_terminal_recv_task(void* parameter)
{
  static const char* TAG = "HAND_TERMINAL_RECV_TASK";
  while (1)
  {
    /* TODO: should this variables be created once? */
    /* Please ref espidf_framework_test/test_only.espidf_select_cpu */
    fd_set read_fds;
    struct timeval timeout;
    int rc;

    // Initialize file descriptor set
    FD_ZERO(&read_fds);
    FD_SET(_udp_socket, &read_fds);

    // Set timeout
    timeout.tv_sec = 10;
    timeout.tv_usec = 0;

    // Use select to wait for data
    rc = select(_udp_socket + 1, &read_fds, NULL, NULL, &timeout);

    if (rc < 0)
    {
      ESP_LOGE(TAG, "Select failed: errno %d", errno);
      break;
    }
    else if (rc == 0)
    {
      ESP_LOGD(TAG, "Select timeout, no data received");
    }
    else
    {
      if (FD_ISSET(_udp_socket, &read_fds))
      {
        // Receive UDP message
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        char rx_buffer[128];
        int len = recvfrom(_udp_socket, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr*)&source_addr, &socklen);
        if (len > 0)
        {
          rx_buffer[len] = 0;  // Null-terminate whatever we received

          /* XXX: just for demo, change to groupbit later */
          ESP_LOGI(TAG, "Got: %s", rx_buffer);

          /* TODO: add circular buffer and it's api */
        }
      }
    }
  }
}

/* Public API */
esp_err_t hand_terminal_module_mount(void* parameter)
{
  /* TODO: implement */
  ESP_LOGW(TAG, "No implementation yet");
  return ESP_OK;
}

esp_err_t hand_terminal_module_unmount(void* parameter)
{
  /* TODO: implement */
  ESP_LOGW(TAG, "No implementation yet");
  return ESP_OK;
}

esp_err_t hand_terminal_module_init(hand_terminal_handle_t handle)
{
  /* TODO: impl handle verification */

  /* create server */
  esp_err_t ret = hand_terminal_module_create_server(&handle->local_server,
                                                     &handle->dest_addr);
  ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

  if (ret != ESP_OK)
  {
    return ret;
  }

  /* overwrite esp log vprintf (out direction) */
  ESP_LOGW(TAG, "Change log vprintf to `hand_esp_log_handler_vprintf`");
  _original_vprintf = esp_log_set_vprintf(hand_esp_log_handler_vprintf);

  /* run terminal input task (in direction) */
  /* TODO: add para for recv task */
  void* para = NULL;
  xTaskCreate(hand_terminal_recv_task, "hand_terminal_recv_task",
              HAND_TASK_SS_TERMINAL_RECV, para,
              HAND_TASK_PRIORITY_TERMINAL_RECV, &_recv_task_handle);

  /* send esp32 ip and port information to destination server side */
  esp_netif_ip_info_t ip_info;
  hand_server_get_current_ip(&ip_info);

  ESP_LOGI("HAND_IP_INFO", "[" IPSTR ", %d]", IP2STR(&ip_info.ip),
           handle->local_server.addr.port);
  ESP_LOGW(TAG, "HAND system can now recv user command!");

  return ESP_OK;
}

esp_err_t hand_terminal_module_deinit(hand_terminal_handle_t handle)
{
  esp_err_t ret = ESP_OK;

  ESP_LOGW(TAG, "Restore original log vprintf");
  if (_original_vprintf != NULL)
  {
    esp_log_set_vprintf(_original_vprintf);
  }

  /* delete recv task */
  if (_recv_task_handle != NULL)
  {
    vTaskDelete(_recv_task_handle);
    _recv_task_handle = NULL;
  }

  /* delete created server */
  ret = hand_terminal_module_destroy_server(&handle->local_server);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to destroy local server: %s", esp_err_to_name(ret));
    return ret;
  }

  return ret;
}