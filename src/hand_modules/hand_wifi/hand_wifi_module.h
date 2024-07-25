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
#include <stdint.h>

#include "esp_err.h"
#include "esp_wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* Public define */
#define HAND_WIFI_MODULE_DEFAULT_SSID      ("CHT061975")
#define HAND_WIFI_MODULE_DEFAULT_PASSWORD  ("24577079")
#define HAND_WIFI_MODULE_DEFAULT_MAX_RETRY (5)
#define HAND_WIFI_MODULE_DEFAULT_MODE      (WIFI_MODE_STA)
#define HAND_WIFI_MODULE_DEFAULT_INTERFACE (WIFI_IF_STA)

#define HAND_WIFI_MODULE_SSID_MAX_LEN     (32)
#define HAND_WIFI_MODULE_PASSWORD_MAX_LEN (64)

/* hand wifi event group bit definition */
#define HAND_WIFI_CONNECTED_BIT    (BIT0)
#define HAND_WIFI_CONNECT_FAIL_BIT (BIT1)
#define HAND_WIFI_GOT_IP_BIT       (BIT2)

/* Public struct */
typedef struct hand_wifi_config_t
{
  uint8_t ssid_len;
  uint8_t password_len;
  uint8_t ssid[HAND_WIFI_MODULE_SSID_MAX_LEN];
  uint8_t password[HAND_WIFI_MODULE_PASSWORD_MAX_LEN];
} hand_wifi_config_t;

typedef struct hand_wifi_t
{
  /* pure number */
  uint8_t max_retry;

  /* enum */
  wifi_interface_t interface;
  wifi_mode_t mode;

  /* struct ptr */
  // TODO: make the whole interface can function both static allocate or dynamic
  // allocate
  /**
   * @brief The event group that monitor the wifi event
   *
   * @details If the event group is NULL during initialization, it will be
   * created internally, and the `_internal_create_event_group` flag will be set
   * to ensure it is cleaned up during deinitialization.
   *
   * @note
   * - BIT0: HAND_WIFI_CONNECTED_BIT
   * @note
   * - BIT1: HAND_WIFI_FAIL_BIT
   * @note
   * - BIT2: HAND_WIFI_GOT_IP_BIT
   *
   * @warning Currently, only allow user to use heap allocate
   * (xEventGroupCreate)
   *
   *
   */
  EventGroupHandle_t wifi_event_group;

  /* struct */
  /**
   * @brief Wi-Fi SSID and password and related len
   *
   */
  hand_wifi_config_t config;

  /* user defined */
  void* user;
} hand_wifi_t;

typedef hand_wifi_t* hand_wifi_handle_t;

/* Public API */

/**
 * @brief 建議呼叫順序:
 * 1. hand_wifi_module_mount
 * 2. 創建 handle_wifi_t, 可以使用 hand_wifi_module_get_default_handle 獲取預設
 * handle
 * 3. hand_wifi_module_update_handler, 更新客製的事件處理
 * 4. hand_wifi_module_init, 呼叫初始化，並根據 wait_util_connect 確認是否等待
 * block 到連線完成
 * 5. hand_wifi_module_deinit
 * 6. hand_wifi_module_unmount
 *
 * hand_wifi_module_get_event_group_handle 可以從外部獲取 group_event_bit
 * 目前的情況
 */

/**
 * @brief
 *
 * @param parameter
 * @return esp_err_t
 */
esp_err_t hand_wifi_module_mount(void* parameter);

esp_err_t hand_wifi_module_unmount(void* parameter);

esp_err_t hand_wifi_module_init(hand_wifi_handle_t handle,
                                bool wait_util_connect);

// TODO: 1. 需要檢查目前的 group event 地址是否更動
//          - 如果是跟目前一樣且是內部產生 -> 不刪
//          - 跟目前不一樣且是內部產生-> 刪再改
//          - 不是內部產生 -> 跳過
/**
 * @brief
 *
 * @param handle
 * @return esp_err_t
 */
esp_err_t hand_wifi_module_update(hand_wifi_handle_t handle);

esp_err_t hand_wifi_module_connect();

esp_err_t hand_wifi_module_disconnect();

/**
 * @brief Replace custom wifi event handler
 *
 * @warning Please do not "reset the set bits" in the event_handler, which will
 * break the internal process.
 */
esp_err_t hand_wifi_module_update_handler(esp_event_handler_t event_handler);

esp_err_t hand_wifi_module_deinit(void* parameter);

EventGroupHandle_t hand_wifi_module_get_event_group_handle();

/* user needs to allocate the dereference of handle first */
esp_err_t hand_wifi_module_get_default_handle(hand_wifi_handle_t handle);
