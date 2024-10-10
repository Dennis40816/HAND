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
#include "hand_wifi/hand_wifi_module.h"

#include "sys/socket.h"

#include "hand_server/hand_server.h"

/* Private variables */

/* Public define */
#define HAND_TERMINAL_MODULE_DEFAULT_IP       ("192.168.1.17")
#define HAND_TERMINAL_MODULE_DEFAULT_PORT     (12345)
#define HAND_TERMINAL_MODULE_ADDRESS_INFO_STR ("[" IPSTR "],[%d]");

typedef struct hand_terminal_t
{
  hand_server_config_t local_server;
  hand_server_address_t dest_addr;
} hand_terminal_t;

typedef hand_terminal_t* hand_terminal_handle_t;

/* Public API */
esp_err_t hand_terminal_module_mount(void* parameter);

esp_err_t hand_terminal_module_unmount(void* parameter);

/**
 * @brief
 *
 * @note This function will create internal modules listed below:
 * @note - a udp server : receive user input data from PC/smartphone and
 * transmit log information to user side
 * @note - a receiver task:
 *
 * @param handle
 * @return esp_err_t
 */
esp_err_t hand_terminal_module_init(hand_terminal_handle_t handle);

esp_err_t hand_terminal_module_deinit(hand_terminal_handle_t handle);
