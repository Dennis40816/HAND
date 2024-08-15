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

#include "hand_config.h"
// hand_wifi_module can be overwritten by hand_config.h
#include "hand_wifi/hand_wifi_module.h"
#include "hand_terminal/hand_terminal_module.h"

#include "stdbool.h"

#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* devices related */
#include "bos1901.h"
#include "vl53l1x.h"
#include "tca6408a.h"
#include "board/hand_chx01.h"

/* third party library */
#include "led_strip.h"

/* Public struct */

/**
 * @brief The control handles of all devices in the HAND system are stored in a
 * structure.
 *
 */
typedef struct hand_devices_handle_t
{
  /* TODO: CH-101, make it pointer */
  ch_group_t ch101_group;
  ch_dev_t ch101_dev[HAND_DEV_MAX_NUM_CH101];

  /* TODO: TCA6408A, make it pointer */
  /**
   * @brief TCA6408A device array
   * 
   * @note - index 0: TCA6408A OTHER (@I2C_0)
   * @note - index 1: TCA6408A CH101 (@I2C_1)
   */
  tca6408a_dev_t tca_dev[HAND_DEV_MAX_NUM_TCA6408];

  /* TODO: BMI323, need refactor */
  spi_device_handle_t bmi323_handle;

  /* TODO: BQ27427, waiting for implementation */

  /* TODO: BOS1901, make it array */
  bos1901_dev_handle_t bos1901_1_dev_handle;
  bos1901_dev_handle_t bos1901_2_dev_handle;
  bos1901_dev_handle_t bos1901_3_dev_handle;
  bos1901_dev_handle_t bos1901_4_dev_handle;

  // TODO: Private, user should not access these. Need refactor also make it
  // array
  spi_device_handle_t bos1901_1_spi_handle;
  spi_device_handle_t bos1901_2_spi_handle;
  spi_device_handle_t bos1901_3_spi_handle;
  spi_device_handle_t bos1901_4_spi_handle;

  /* TODO: KX132-1211, need refactor. Make it array */
  spi_device_handle_t kx132_1_dev_handle;
  spi_device_handle_t kx132_2_dev_handle;
  spi_device_handle_t kx132_3_dev_handle;
  spi_device_handle_t kx132_4_dev_handle;

  /* TODO: VL53L1X, make it a pointer */
  VL53L1_Dev_t vl53l1x_dev[HAND_DEV_MAX_NUM_VL53L1X];

  /* TODO: RGB LED */
  led_strip_handle_t rgb_led_handle;

} hand_devices_handle_t;

typedef struct hand_global_states_t
{
  /* semaphore managements (data synchronization) */

  /* binary states */
  bool hand_usb_plug_state;  // default to HAND_VBUS_DETECT_UNPLUG
  bool hand_bat_plug_state;  // default to HAND_BATTERY_DETECT_UNPLUG

  /* multiple states */

} hand_global_states_t;

/* Public API */
esp_err_t hand_init(const char* ssid, const char* password, bool init_dev);