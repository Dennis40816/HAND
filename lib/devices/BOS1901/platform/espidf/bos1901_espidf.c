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

/* espidf related headers */
#include "esp_err.h"

#include "bos1901_espidf.h"
#include "bos1901.h"

static const char *TAG = "bos1901_espidf.c";

/**
 * @brief Converts platform-specific error codes to unified BOS1901 error codes.
 *
 * This function provides a way to translate platform-specific error codes
 * into the unified BOS1901 error codes defined in `bos1901_err_t`.
 *
 * @param platform_err The platform-specific error code.
 * @return bos1901_err_t The corresponding unified BOS1901 error code.
 */
static bos1901_err_t convert_platform_error(int platform_err);

/**
 * @brief Initializes the SPI device for ESP-IDF platform.
 *
 * This function sets up the SPI device configuration and registers it with the
 * ESP-IDF SPI bus.
 *
 * @param config Pointer to the ESP-IDF SPI device configuration.
 */
void bos1901_espidf_spi_init_device(bos1901_espidf_spi_dev_config_t *config)
{
  esp_err_t ret = spi_bus_add_device(config->target_bus, &config->dev_config,
                                     config->handle_ptr);
  if (ret == ESP_OK)
  {
    ESP_LOGD(TAG, "handle's address is: %p", (void *)config->handle_ptr);
    ESP_LOGI(TAG, "SPI bus add device ok!");
  }
  else
  {
    ESP_LOGE(TAG, "SPI bus add device failed!");
  }
}

/**
 * @brief Deinitializes the SPI device for ESP-IDF platform.
 *
 * This function removes the SPI device from the ESP-IDF SPI bus.
 *
 * @param config Pointer to the ESP-IDF SPI device configuration.
 */
void bos1901_espidf_spi_deinit_device(bos1901_espidf_spi_dev_config_t *config)
{
  spi_bus_remove_device(*config->handle_ptr);
}

/**
 * @brief Performs a read-write operation on the SPI device.
 *
 * This function writes a 16-bit word to the SPI device and reads the response.
 * The data is converted to and from big-endian format.
 *
 * @param config Pointer to the ESP-IDF SPI device configuration.
 * @param data The data word to be written.
 * @return uint16_t The read data word.
 */
bos1901_err_t bos1901_espidf_read_write_word(
    bos1901_espidf_spi_dev_config_t *config, uint16_t data_tx,
    uint16_t *data_rx)
{
  ESP_LOGD(TAG, "handle's address is: %p", (void *)*config->handle_ptr);
  uint16_t data_tx_r = __builtin_bswap16(data_tx);
  uint16_t data_rx_r = 0;

  spi_transaction_t transaction = {
      .length = 16,  // Transfer 16 bits
      .tx_buffer = &data_tx_r,
      .rx_buffer = &data_rx_r,
  };
  esp_err_t ret = spi_device_transmit(*config->handle_ptr, &transaction);

  /* users might use NULL to show data_rx is not important to them */
  if (data_rx != NULL)
  {
    ESP_LOGV(TAG, "User set data_rx NULL");
    *data_rx = __builtin_bswap16(data_rx_r);
  }
  return convert_platform_error(ret);
}

/**
 * @brief Performs a read-write operation on the SPI device for an array of
 * words.
 *
 * This function writes an array of 16-bit words to the SPI device and reads the
 * response into the provided buffer. The data is converted to and from
 * big-endian format.
 *
 * @param config Pointer to the ESP-IDF SPI device configuration.
 * @param data_tx Pointer to the data array to be transmitted.
 * @param data_rx Pointer to the buffer where the received data will be stored.
 * @param length Number of words to transfer.
 * @return bos1901_err_t The result of the read-write operation.
 */
bos1901_err_t bos1901_espidf_read_write(bos1901_espidf_spi_dev_config_t *config,
                                        uint16_t *data_tx, uint16_t *data_rx,
                                        uint32_t length)
{
  // Convert data to big-endian before transmission
  for (uint32_t i = 0; i < length; i++)
  {
    data_tx[i] = __builtin_bswap16(data_tx[i]);
  }

  spi_transaction_t trans = {
      .length = length * 16,  // length in bits
      .tx_buffer = data_tx,
      .rx_buffer = data_rx,
  };

  bos1901_err_t err =
      convert_platform_error(spi_device_transmit(*config->handle_ptr, &trans));

  if (err == BOS1901_OK)
  {
    if (data_rx != NULL)
    {
      // Convert received data from big-endian
      for (uint32_t i = 0; i < length; i++)
      {
        data_rx[i] = __builtin_bswap16(data_rx[i]);
      }
    }
  }
  return err;
}

/**
 * @brief Provides the default SPI interface for the BOS1901 device on ESP-IDF
 * platform.
 *
 * This function returns a structure containing pointers to functions that
 * handle SPI initialization, deinitialization, reading, and writing.
 *
 * @return bos1901_spi_interface_t The default SPI interface.
 */
bos1901_spi_interface_t bos1901_get_default_spi_interface()
{
  bos1901_spi_interface_t spi_interface = {
      .init_device = bos1901_espidf_spi_init_device,
      .deinit_device = bos1901_espidf_spi_deinit_device,
      .read_write_word = bos1901_espidf_read_write_word,
      .read_write = bos1901_espidf_read_write,
      .name = "default_espidf_spi_interface",
      .description = "Default BOS1901 SPI interface for ESP-IDF platform."};
  return spi_interface;
}

/* Private functions */

/**
 * @brief Converts ESP-IDF error codes to BOS1901 unified error codes.
 *
 * This function maps ESP-IDF specific error codes to the unified
 * BOS1901 error codes defined in `bos1901_err_t`.
 *
 * @param platform_err The ESP-IDF specific error code.
 * @return bos1901_err_t The corresponding BOS1901 error code.
 */
static bos1901_err_t convert_platform_error(int platform_err)
{
  switch (platform_err)
  {
    case ESP_OK:
      return BOS1901_OK;
    case ESP_ERR_INVALID_ARG:
      return BOS1901_INVALID_ARG;
    case ESP_ERR_INVALID_STATE:
    case ESP_FAIL:
      return BOS1901_ERR_BUS;
    case ESP_ERR_TIMEOUT:
      return BOS1901_TIMEOUT;
    default:
      return BOS1901_UNKNOWN;
  }
}
