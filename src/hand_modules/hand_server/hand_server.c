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

#include "hand_server.h"
#include "esp_log.h"

static const char* TAG = "HAND_SERVER";

/**
 * @brief Get the current IP address of the server.
 *
 * This function retrieves the current IP address assigned to the server.
 *
 * @param info Pointer to the esp_netif_ip_info_t structure where the IP
 * information will be stored.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t hand_server_get_current_ip(esp_netif_ip_info_t* info)
{
  return esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"),
                               info);
}

/**
 * @brief Create a server with the specified configuration.
 *
 * This function creates a server (TCP or UDP) with the given configuration and
 * binds it to the specified address.
 *
 * @param local_config Pointer to the hand_server_config_t structure containing
 * the server configuration.
 * @param dest_addr Pointer to the hand_server_address_t structure containing
 * the destination address.
 * @param socket_fd Pointer to the socket descriptor to be returned.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t hand_server_create(hand_server_config_t* local_config,
                             hand_server_address_t* dest_addr, int* socket_fd)
{
  if (local_config->addr_family == HAND_AF_INET6)
  {
    ESP_LOGE(TAG,
             "Currently, there's no implementation for INET6 address! Abort");
    return ESP_ERR_NOT_SUPPORTED;
  }

  struct sockaddr_in _dest_addr;
  if (local_config->addr_family == HAND_AF_INET)
  {
    _dest_addr.sin_addr.s_addr = inet_addr(dest_addr->ip);
    _dest_addr.sin_family = local_config->addr_family;
    _dest_addr.sin_port = htons(dest_addr->port);
  }

  int tmp_socket = -1;
  if (local_config->server_type == HAND_UDP_SERVER)
  {
    tmp_socket = socket(local_config->addr_family, SOCK_DGRAM, IPPROTO_UDP);
    if (tmp_socket < 0)
    {
      ESP_LOGE(TAG, "Create UDP socket failed, socket value is: {%d}",
               tmp_socket);
      return ESP_ERR_INVALID_ARG;
    }
  }
  else if (local_config->server_type == HAND_TCP_SERVER)
  {
    tmp_socket = socket(local_config->addr_family, SOCK_STREAM, IPPROTO_TCP);
    if (tmp_socket < 0)
    {
      ESP_LOGE(TAG, "Create TCP socket failed, socket value is: {%d}",
               tmp_socket);
      return ESP_ERR_INVALID_ARG;
    }
  }
  else
  {
    ESP_LOGE(TAG, "Invalid server type: %d", local_config->server_type);
    return ESP_ERR_INVALID_ARG;
  }

  struct sockaddr_in local_addr;
  local_addr.sin_family = local_config->addr_family;
  local_addr.sin_addr.s_addr = INADDR_ANY;
  local_addr.sin_port = htons(local_config->addr.port);
  if (bind(tmp_socket, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0)
  {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
    close(tmp_socket);
    return ESP_ERR_INVALID_ARG;
  }

  if (local_config->server_type == HAND_TCP_SERVER)
  {
    if (listen(tmp_socket, local_config->max_connection) < 0)
    {
      ESP_LOGE(TAG, "Socket listen failed: errno %d", errno);
      close(tmp_socket);
      return ESP_ERR_INVALID_ARG;
    }
  }

  fcntl(tmp_socket, F_SETFL, local_config->fcntl_flag);

  esp_netif_ip_info_t ip_info;
  esp_err_t ret = hand_server_get_current_ip(&ip_info);
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Got current IP failed due to error code: {%d}", ret);
    close(tmp_socket);
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG,
           "Server created successfully. Bound to IP: { " IPSTR " }, port: %d",
           IP2STR(&ip_info.ip), local_config->addr.port);

  *socket_fd = tmp_socket;
  return ESP_OK;
}

/* TODO: make it nonblock will be better */

/**
 * @brief Create a client socket and onnect to the server using the specified
 * configuration. Currently, only for TCP client only.
 *
 * @param client_config Pointer to the client configuration structure.
 * @param client_socket Pointer to the socket descriptor to be returned.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 * 
 * @warning This function perform connection in blocking mode
 */
esp_err_t hand_client_connect(hand_tcp_client_config_t* client_config,
                              int* client_socket)
{
  if (client_config->addr_family == HAND_AF_INET6)
  {
    ESP_LOGE(TAG,
             "Currently, there's no implementation for INET6 address! Abort");
    return ESP_ERR_NOT_SUPPORTED;
  }

  int sock = socket(client_config->addr_family, SOCK_STREAM, IPPROTO_TCP);
  if (sock < 0)
  {
    ESP_LOGE(TAG, "Failed to create socket, errno: %d", errno);
    return ESP_ERR_INVALID_ARG;
  }

  struct sockaddr_in server_addr;
  server_addr.sin_family = client_config->addr_family;
  server_addr.sin_port = htons(client_config->server_addr.port);
  if (inet_pton(AF_INET, client_config->server_addr.ip,
                &server_addr.sin_addr) <= 0)
  {
    ESP_LOGE(TAG, "Invalid address/ Address not supported: %s",
             client_config->server_addr.ip);
    close(sock);
    return ESP_ERR_INVALID_ARG;
  }

  if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
  {
    ESP_LOGE(TAG, "Connection failed, errno: %d", errno);
    close(sock);
    return ESP_ERR_INVALID_ARG;
  }

  fcntl(sock, F_SETFL, client_config->fcntl_flag);

  ESP_LOGI(TAG, "Connected to server at %s:%d as client '%s'",
           client_config->server_addr.ip, client_config->server_addr.port,
           client_config->client_name);

  *client_socket = sock;
  return ESP_OK;
}