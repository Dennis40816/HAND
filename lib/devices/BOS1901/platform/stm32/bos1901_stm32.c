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

#include "bos1901.h"
#include "bos1901_stm32.h"

/**
 * @brief Converts platform-specific error codes to unified BOS1901 error
 * codes.
 *
 * This function provides a way to translate platform-specific error codes
 * into the unified BOS1901 error codes defined in `bos1901_err_t`.
 *
 * @param platform_err The platform-specific error code.
 * @return bos1901_err_t The corresponding unified BOS1901 error code.
 */
static bos1901_err_t convert_platform_error(int platform_err);

void bos1901_stm32_hal_spi_init_device(bos1901_spi_dev_config_t *config)
{
  // stm32 hal specific initialization logic, if needed
}

void bos1901_stm32_hal_spi_deinit_device(bos1901_spi_dev_config_t *config)
{
  // stm32 hal specific de-initialization logic, if needed
}

bos1901_err_t bos1901_stm32_hal_spi_write(bos1901_spi_dev_config_t *config,
                                          uint8_t *data, size_t length)
{
  // pull cs pin low
  HAL_GPIO_WritePin(config->cs_pin.gpio_x, config->cs_pin.gpio_pin,
                    GPIO_PIN_RESET);
  int status = HAL_SPI_Transmit(config->handle, data, length, HAL_MAX_DELAY);
  // pull cs pin high
  HAL_GPIO_WritePin(config->cs_pin.gpio_x, config->cs_pin.gpio_pin,
                    GPIO_PIN_SET);
  return convert_platform_error(status);
}

bos1901_err_t bos1901_stm32_hal_spi_read(bos1901_spi_dev_config_t *config,
                                         uint8_t *data, size_t length)
{
  // pull cs pin low
  HAL_GPIO_WritePin(config->cs_pin.gpio_x, config->cs_pin.gpio_pin,
                    GPIO_PIN_RESET);
  int status = HAL_SPI_Receive(config->handle, data, length, HAL_MAX_DELAY);
  // pull cs pin high
  HAL_GPIO_WritePin(config->cs_pin.gpio_x, config->cs_pin.gpio_pin,
                    GPIO_PIN_SET);
  return convert_platform_error(status);
}

bos1901_spi_interface_t bos1901_get_default_spi_interface()
{
  bos1901_spi_interface_t spi_interface_ptr = {
      .init_device = bos1901_stm32_hal_spi_init_device,
      .deinit_device = bos1901_stm32_hal_spi_deinit_device,
      .read = bos1901_stm32_hal_spi_read,
      .write = bos1901_stm32_hal_spi_write,
      .name = "default_stm32_spi_interface",
      .description = "default bos1901 spi interface for STM32 platform."};
  return spi_interface_ptr;
}

/* private functions */

/**
 * @brief Converts STM32 error codes to BOS1901 unified error codes.
 *
 * This function maps STM32 specific error codes to the unified
 * BOS1901 error codes defined in `bos1901_err_t`.
 *
 * @param platform_err The STM32 specific error code.
 * @return bos1901_err_t The corresponding BOS1901 error code.
 */
static bos1901_err_t convert_platform_error(int platform_err)
{
  switch (platform_err)
  {
    case HAL_OK:
      return BOS1901_OK;
    case HAL_ERROR:
      return BOS1901_ERR_BUS;
    case HAL_BUSY:
      return BOS1901_BUSY;
    case HAL_TIMEOUT:
      return BOS1901_TIMEOUT;
    default:
      return BOS1901_UNKNOWN;
  }
}