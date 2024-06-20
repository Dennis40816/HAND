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

#ifndef LIB_BOS1901_PLATFORM_STM32_BOS1901_STM32_H_
#define LIB_BOS1901_PLATFORM_STM32_BOS1901_STM32_H_

#include <stdint.h>

/* bos1901 related header */
#include "bos1901_err.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C"
{
#endif

  typedef struct bos1901_stm32_io_port_t
  {
    GPIO_TypeDef *gpio_x;  // gpio port
    uint16_t gpio_pin;     // gpio pin
  } bos1901_stm32_io_port_t;

  /* alias io bos1901_io_port_t */
  typedef bos1901_stm32_io_port_t bos1901_io_port_t;

  /**
   * @brief STM32 HAL specific SPI device configuration.
   * This structure contains the handle and chip select pin configuration
   * specific to STM32 HAL.
   */
  typedef struct
  {
    SPI_HandleTypeDef *handle;  // stm32 hal specific handle
    bos1901_stm32_io_port_t cs_pin;
  } bos1901_stm32_hal_spi_dev_config_t;

  /* alias to bos1901_spi_dev_config_t */
  typedef bos1901_stm32_hal_spi_dev_config_t bos1901_spi_dev_config_t;

#ifdef __cplusplus
}
#endif

#endif