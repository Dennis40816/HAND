#pragma once

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
#include "sys/socket.h"
#include "fcntl.h"

#include "esp_netif.h"

/* Public struct */
/**
 * @brief This enum is related to the socket's IP protocol options.
 *
 */
typedef enum hand_server_type_t
{
  /**
   * @brief Corresponds to IPPROTO_TCP
   *
   */
  HAND_TCP_SERVER,

  /**
   * @brief Corresponds to IPPROTO_UDP
   *
   */
  HAND_UDP_SERVER,

  HAND_SERVER_MAX,
} hand_server_type_t;

/**
 * @brief This enum is related to the socket's address family options.
 *
 */
typedef enum hand_socket_addr_family_t
{
  /**
   * @brief IPv4 address family
   *
   */
  HAND_AF_INET = AF_INET,

  /**
   * @brief IPv6 address family
   *
   */
  HAND_AF_INET6 = AF_INET6,
} hand_socket_addr_family_t;

typedef struct hand_server_address_t
{
  /**
   * @brief Server IP address
   *
   * A string representing the IP address of the server.
   */
  char ip[INET6_ADDRSTRLEN];  // Enough space for both IPv4 and IPv6
                              // addresses

  /**
   * @brief Server port number
   *
   * The port number on which the server will listen for incoming connections.
   */
  uint16_t port;
} hand_server_address_t;

/**
 * @brief Configuration structure for the server.
 *
 * This structure contains configuration settings for the server, including
 * the type of server (TCP/UDP), address family (IPv4/IPv6), socket flags,
 * server IP address, and server port.
 */
typedef struct hand_server_config_t
{
  /**
   * @brief Related to IP protocol option (e.g., IPPROTO_TCP)
   *
   * Specifies whether the server is a TCP or UDP server.
   */
  hand_server_type_t server_type;

  /**
   * @brief Max client number at once
   * @warning Only take effect when using TCP server
   *
   */
  int max_connection;

  /**
   * @brief Address family option (e.g., AF_INET for IPv4, AF_INET6 for IPv6)
   *
   * Specifies the address family to be used (IPv4 or IPv6).
   */
  hand_socket_addr_family_t addr_family;

  /**
   * @brief Flags for fcntl to set socket options (e.g., O_NONBLOCK)
   *
   * This integer sets flags for the socket using fcntl.
   */
  int fcntl_flag;

  /**
   * @brief Address for server
   *
   * @note If addr.ip is not null -> will treated as require static ip (not
   * implemented)
   */
  hand_server_address_t addr;

} hand_server_config_t;

/* TODO: needs refactor */
/**
 * @brief Configuration structure for the client.
 *
 * This structure contains configuration settings for the client, including
 * the address family (IPv4/IPv6), socket flags, server IP address, server port,
 * and client name.
 */
typedef struct hand_tcp_client_config_t
{
  /**
   * @brief Address family option (e.g., AF_INET for IPv4, AF_INET6 for IPv6)
   *
   * Specifies the address family to be used (IPv4 or IPv6).
   */
  hand_socket_addr_family_t addr_family;

  /**
   * @brief Flags for fcntl to set socket options (e.g., O_NONBLOCK)
   *
   * This integer sets flags for the socket using fcntl.
   */
  int fcntl_flag;

  /**
   * @brief Address for server
   *
   * Specifies the IP address and port of the server to connect to.
   */
  hand_server_address_t server_addr;

  /**
   * @brief Name of the client
   *
   * A string representing the name of the client.
   */
  const char* client_name;
} hand_tcp_client_config_t;

/* public API */

esp_err_t hand_server_get_current_ip(esp_netif_ip_info_t* info);
esp_err_t hand_server_create(hand_server_config_t* local_config,
                             hand_server_address_t* dest_addr, int* socket_fd);

/* TODO: needs refactor */

esp_err_t hand_client_connect(hand_tcp_client_config_t* client_config,
                              int* client_socket);