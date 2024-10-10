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

#include <stdint.h>

#include "bos1901.h"
#include "bos1901_dummy.h"

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

__attribute__((weak)) void bos1901_dummy_spi_init_device(
    bos1901_spi_dev_config_t *config)
{
}

__attribute__((weak)) void bos1901_dummy_spi_deinit_device(
    bos1901_spi_dev_config_t *config)
{
}

__attribute__((weak)) bos1901_err_t bos1901_dummy_spi_write(
    bos1901_spi_dev_config_t *config, uint8_t *data, uint32_t length)
{
  return BOS1901_OK;  // return success
}

__attribute__((weak)) bos1901_err_t bos1901_dummy_spi_read(
    bos1901_spi_dev_config_t *config, uint8_t *data, uint32_t length)
{
  return BOS1901_OK;  // return success
}

__attribute__((weak)) bos1901_spi_interface_t
bos1901_get_default_spi_interface()
{
  bos1901_spi_interface_t spi_interface_ptr = {
      .init_device = NULL,
      .deinit_device = NULL,
      .read = NULL,
      .write = NULL,
      .name = "default_dummy_spi_interface",
      .description = "default bos1901 spi interface for compiling test only!!"};
  return spi_interface_ptr;
}

/**
 * @brief Converts ESP-IDF error codes to BOS1901 unified error codes.
 *
 * This function maps ESP-IDF specific error codes to the unified
 * BOS1901 error codes defined in `bos1901_err_t`.
 *
 * @param platform_err The DUMMY specific error code.
 * @return bos1901_err_t The corresponding BOS1901 error code.
 */
static bos1901_err_t convert_platform_error(int platform_err)
{
  switch (platform_err)
  {
    default:
      return BOS1901_OK;
  }
}