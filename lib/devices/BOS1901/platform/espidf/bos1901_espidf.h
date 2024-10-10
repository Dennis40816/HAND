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

#ifndef LIB_BOS1901_PLATFORM_ESPIDF_BOS1901_ESPIDF_H_
#define LIB_BOS1901_PLATFORM_ESPIDF_BOS1901_ESPIDF_H_

#include <stdint.h>

/* espidf related header */
#include "driver/spi_master.h"
#include "esp_log.h"

/* bos1901 related header */
#include "bos1901_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief ESP-IDF specific SPI device configuration.
   * This structure contains the handle and device configuration specific to
   * ESP-IDF.
   */
  typedef struct bos1901_espidf_spi_dev_config_t
  {
    spi_host_device_t target_bus;              // esp-idf target bus
    spi_device_handle_t* handle_ptr;           // esp-idf specific handle
    spi_device_interface_config_t dev_config;  // esp-idf device config
  } bos1901_espidf_spi_dev_config_t;

  /* alias to bos1901_spi_dev_config_t */
  typedef bos1901_espidf_spi_dev_config_t bos1901_spi_dev_config_t;

/* alias BOS_LOG */
#define BOS_LOGI ESP_LOGI
#define BOS_LOGW ESP_LOGW
#define BOS_LOGE ESP_LOGE
#define BOS_LOGD ESP_LOGD
#define BOS_LOGV ESP_LOGV

#ifdef __cplusplus
}
#endif

#endif