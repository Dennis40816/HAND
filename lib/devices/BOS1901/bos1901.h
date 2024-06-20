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

#ifndef LIB_BOS1901_BOS1901_H_
#define LIB_BOS1901_BOS1901_H_

#include <stdint.h>
#include "bos1901_err.h"
#include "bos1901_config.h"

#ifdef LIB_USE_ESPIDF_PLATFORM
#include "platform/espidf/bos1901_espidf.h"
#pragma message("LIB_USE_ESPIDF_PLATFORM defined")
#elif defined LIB_USE_STM32_PLATFORM
#include "platform/stm32/bos1901_stm32.h"
#pragma message("LIB_USE_STM32_PLATFORM defined")
#elif defined LIB_USE_DUMMY_PLATFORM
#include "platform/dummy/bos1901_dummy.h"
#pragma message("LIB_USE_DUMMY_PLATFORM defined")
#else
#pragma error("no platform defined")
#endif

#ifdef __cplusplus
extern "C"
{
#endif

  /* TODO: v2 中要思考把這些定義放在哪裡，才能讓 config 相關, device operation,
   * spi operation, 等不同函式分開放置到不同的檔案 */

  /**
   * We define a bos1901 register (uint16_t, 2 bytes) like following:
   *
   * xxxx xxxx xxxx xxxx
   *
   * first 4 bits: address
   * last 12 bits: data
   *
   */

  /**
   * @brief SPI interface structure for BOS1901.
   *
   * This structure defines a set of function pointers for SPI operations.
   */
  typedef struct bos1901_spi_interface_t
  {
    const char *name;
    const char *description;
    void (*init_device)(
        bos1901_spi_dev_config_t *config);  ///< Device initialization function
    void (*deinit_device)(bos1901_spi_dev_config_t
                              *config);  ///< Device de-initialization function
    bos1901_err_t (*read_write_word)(
        bos1901_spi_dev_config_t *config, uint16_t data_tx,
        uint16_t *data_rx);  ///< Read/write word function
    bos1901_err_t (*read_write)(bos1901_spi_dev_config_t *config,
                                uint16_t *data_tx, uint16_t *data_rx,
                                uint32_t length);
  } bos1901_spi_interface_t;

  /**
   * @brief SPI device structure for BOS1901.
   *
   * This structure holds the SPI interface and its configuration for BOS1901.
   */
  typedef struct bos1901_spi_dev_t
  {
    bos1901_spi_interface_t
        *spi_interface_ptr;  ///< Pointer to the SPI interface
    bos1901_spi_dev_config_t
        *device_config;  ///< Pointer to the device configuration
  } bos1901_spi_dev_t;

  /**
   * @brief BOS1901 device structure.
   *
   * This structure holds the device configuration and the SPI device for
   * BOS1901.
   */
  typedef struct bos1901_dev_t
  {
    const char *dev_name;                     ///< Name of the device
    bos1901_private_status_t private_status;  ///< Store oe, sense status. User
                                              ///< should not modify this
    bos1901_dev_config_t *dev_config;  ///< Pointer to the device configuration
    bos1901_spi_dev_t *spi_dev;        ///< Pointer to the SPI device structure
  } bos1901_dev_t;

  /**
   * @brief Create a new BOS1901 device.
   *
   * @param dev_name Name of the device.
   * @return Pointer to the created BOS1901 device, or NULL on failure.
   */
  bos1901_dev_t *bos1901_device_create(const char *dev_name);

  /**
   * @brief Free the BOS1901 device.
   *
   * @param dev Pointer to the BOS1901 device structure.
   */
  void bos1901_device_free(bos1901_dev_t *dev);

  /**
   * @brief Generate default bos1901 device config struct
   *
   * @return bos1901_dev_config_t
   */
  bos1901_dev_config_t bos1901_get_device_default_config();

  /**
   * @brief Initialize the BOS1901 device.
   *
   * @param dev Pointer to the BOS1901 device structure.
   * @param dev_config Pointer to the platform-independent device configuration.
   * @param spi_device_config Pointer to the SPI device configuration.
   */
  void bos1901_device_init(bos1901_dev_t *dev, bos1901_dev_config_t *dev_config,
                           bos1901_spi_dev_config_t *spi_device_config);

  /**
   * @brief Deinitialize the BOS1901 device.
   *
   * @param dev Pointer to the BOS1901 device structure.
   */
  void bos1901_device_deinit(bos1901_dev_t *dev);

  /**
   * @brief Write data to the BOS1901 device.
   *
   * @param dev Pointer to the BOS1901 device structure.
   * @param reg Target register to write
   * @param data data in word
   * @return bos1901_err_t Status of the write operation.
   */
  bos1901_err_t bos1901_device_write_reg(bos1901_dev_t *dev,
                                         bos1901_register_t reg, uint16_t data);

  /**
   * @brief Read data from the BOS1901 device.
   *
   * @param dev Pointer to the BOS1901 device structure.
   * @param reg Target register to write
   * @param data Pointer to data recv
   * @return bos1901_err_t Status of the read operation.
   */
  bos1901_err_t bos1901_device_read_reg(bos1901_dev_t *dev,
                                        bos1901_register_t reg, uint16_t *data);

  /**
   * @brief Reads and writes a single 16-bit word to the SPI device using
   * the BOS1901 interface.
   *
   * This function performs a read/write operation on the SPI device. The
   * data is transmitted in a 16-bit word format, and the function will swap
   * the byte order to accommodate the endianness differences between the
   * host and the SPI device.
   *
   * @param dev Pointer to the BOS1901 device structure.
   * @param data The 16-bit word to be written to the SPI device.
   * @return The 16-bit word read from the SPI device.
   *
   * @warning Use this only when you know what you're doing. One example is when
   * you are sure that the current value of the CONFIG register's BC is exactly
   * what you need to read, such as when reading the SUP_RISE register. To avoid
   * having bos1901_device_read_reg modify the CONFIG register's BC and then
   * perform a dummy write, you can directly use this function to perform a
   * dummy write to retrieve the SUP_RISE value, thus saving one byte of
   * transmission.
   *
   * Example usage:
   * \code
   * // when you know BC is correct
   * uint16_t dummy = 0xC000
   * uint16_t read_value = bos1901_device_read_write_reg(device, dummy);
   * 
   * printf("Read value: 0x%04X\n", read_value);
   * \endcode
   */
  uint16_t bos1901_device_read_write_reg(bos1901_dev_t *dev, uint16_t data_tx);

  /**
   * @brief Performs a read and write operation on the SPI device using the
   * BOS1901 interface.
   *
   * This function performs a read/write operation on the SPI device for an
   * array of 16-bit words. The data is transmitted and received in a 16-bit
   * word format. The byte order is swapped to handle endianness differences
   * between the host and the SPI device.
   *
   * @param dev Pointer to the BOS1901 device structure.
   * @param data_tx Pointer to the array of 16-bit words to be written to the
   * SPI device.
   * @param data_rx Pointer to the array where the read data will be stored.
   * @param length The number of 16-bit words to be transmitted and received.
   * @return `BOS1901_OK` on success or an appropriate error code.
   *
   * @warning Assume user won't use this function to modify private status field
   *
   * Example usage:
   * \code
   * uint16_t data_tx[] = {0x1234, 0x5678};
   * uint16_t data_rx[2];
   * bos1901_err_t err = bos1901_device_read_write(device, data_tx, data_rx, 2);
   * if (err == BOS1901_OK)
   * {
   *     printf("Read values: 0x%04X, 0x%04X\n", data_rx[0], data_rx[1]);
   * }
   * else
   * {
   *     printf("Error during read/write: %d\n", err);
   * }
   * \endcode
   */
  bos1901_err_t bos1901_device_read_write(bos1901_dev_t *dev, uint16_t *data_tx,
                                          uint16_t *data_rx, uint32_t length);

  /**
   * @brief Change the SPI interface for the BOS1901 device.
   *
   * @param dev Pointer to the BOS1901 device structure.
   * @param new_interface New SPI interface to be set.
   */
  void bos1901_device_change_interface(bos1901_dev_t *dev,
                                       bos1901_spi_interface_t new_interface);

  /* TODO: v1 functions, note that these function should maintain
   * private_status!! */
  // bos1901_get_default_device_config() -> let user get default device config
  bos1901_err_t bos1901_device_output_enable(bos1901_dev_t *dev,
                                             bos1901_oe_status_t new_status);

  /**
   * @brief This function reset bos1901 device, including internal register to
   * default value
   *
   * @param dev Pointer to the BOS1901 device structure.
   * @return bos1901_err_t
   */
  bos1901_err_t bos1901_device_reset(bos1901_dev_t *dev);
  bos1901_err_t bos1901_device_sense_enable(bos1901_dev_t *dev,
                                            bos1901_sense_t new_status);

  /* platfrom related */

  /**
   * @brief Get the default SPI interface.
   *
   * @return The default SPI interface structure.
   */
  bos1901_spi_interface_t bos1901_get_default_spi_interface();

#ifdef __cplusplus
}
#endif

#endif  // LIB_BOS1901_BOS1901_H_
