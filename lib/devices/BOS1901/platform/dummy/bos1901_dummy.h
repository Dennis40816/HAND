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

#ifndef LIB_BOS1901_PLATFORM_DUMMY_BOS1901_DUMMY_H_
#define LIB_BOS1901_PLATFORM_DUMMY_BOS1901_DUMMY_H_

#ifdef __cplusplus
extern "C"
{
#endif

  /**
   * @brief Dummy platform specific SPI device configuration.
   * This structure contains a placeholder handle for dummy implementations.
   */
  typedef struct
  {
    void *handle;  // placeholder for dummy handle
  } bos1901_dummy_spi_dev_config_t;

  /* alias bos1901_spi_dev_config_t */
  typedef bos1901_dummy_spi_dev_config_t bos1901_spi_dev_config_t;

#ifdef __cplusplus
}
#endif
#endif