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

#include "tca6408a.h"
#include "tca6408a_espidf.h"
#include <string.h>

/* high level platform-dependent functions definitions */

/**
 * @brief Converts the register values in the tca6408a_reg_info_t structure to a
 * formatted string.
 *
 * @param info Pointer to the tca6408a_reg_info_t structure containing the
 * register values.
 * @param str Buffer to store the formatted string.
 * @param max_len Maximum length of the buffer.
 * @return tca6408a_err_t Returns ESP_OK if the operation was successful, or
 * ESP_ERR_INVALID_SIZE if the buffer is too small.
 * @note tca6408a_err_t is platform-dependent and could be esp_err_t or other
 * error codes depending on the platform.
 */

// clang-format off
/**
 * @example
 * @code
 * tca6408a_reg_info_t info;
 * char buffer[128];
 * tca6408a_err_t result = tca6408a_reg_info_to_str(&info, buffer, sizeof(buffer));
 * if (result == ESP_OK) {
 *     printf("Register info: \n%s", buffer);
 * }
 * @endcode
 */
// clang-format on
tca6408a_err_t tca6408a_reg_info_to_str(const tca6408a_reg_info_t* info,
                                        char* str, uint32_t max_len)
{
  // fixed template string length
  const char* template =
      "input_reg(0x%02X):    %s\n"
      "output_reg(0x%02X):   %s\n"
      "polarity_reg(0x%02X): %s\n"
      "config_reg(0x%02X):   %s\n";

  // strings to hold the register values
  char input_str[10] = {0};
  char output_str[10] = {0};
  char polarity_str[10] = {0};
  char config_str[10] = {0};

  // convert each register value to a string
  for (int i = 0; i < 8; i++)
  {
    input_str[i] = (info->input_reg & (1 << (7 - i))) ? 'H' : 'L';
    output_str[i] = (info->output_reg & (1 << (7 - i))) ? 'H' : 'L';
    polarity_str[i] = (info->polarity_reg & (1 << (7 - i))) ? 'I' : 'N';
    config_str[i] = (info->config_reg & (1 << (7 - i))) ? 'I' : 'O';
  }

  // final formatted string
  char buffer[200] = {0};
  int length =
      snprintf(buffer, sizeof(buffer), template, info->input_reg, input_str,
               info->output_reg, output_str, info->polarity_reg, polarity_str,
               info->config_reg, config_str);

  if (length >= max_len)
  {
    printf("max length should larger than: %d\n", length);
    return ESP_ERR_INVALID_SIZE;
  }

  strncpy(str, buffer, max_len);

  return ESP_OK;
}

/**
 * @brief Reads all registers of the TCA6408A and stores the values in the
 * provided tca6408a_reg_info_t structure.
 *
 * @param dev_ptr Pointer to the device structure.
 * @param info Pointer to the tca6408a_reg_info_t structure to store the
 * register values.
 * @return tca6408a_err_t Returns ESP_OK if successful, or an error code if the
 * operation failed.
 * @note tca6408a_err_t is platform-dependent and could be esp_err_t or other
 * error codes depending on the platform.
 */

// clang-format off
/**
 * @example
 * @code
 * tca6408a_dev_t dev;
 * tca6408a_reg_info_t info;
 * tca6408a_err_t result = tca6408a_read_all(&dev, &info);
 * if (result == ESP_OK) {
 *     printf("Successfully read all registers.\n");
 * }
 * @endcode
 */
// clang-format on
tca6408a_err_t tca6408a_read_all(const tca6408a_dev_t* dev_ptr,
                                 tca6408a_reg_info_t* info)
{
  tca6408a_err_t ret;
  for (int i = 0; i < TCA6408A_NUM_REGS; i++)
  {
    // because the registers in the info structure are all uint8_t, they can be
    // offset using + i (one byte at a time).
    ret = tca6408a_read_register(dev_ptr, i, &(info->input_reg) + i);
    if (ret != ESP_OK)
    {
      return ret;
    }
  }
  return ret;
}

/**
 * @brief Resets the TCA6408A device.
 *
 * @param dev_ptr Pointer to the device structure.
 * @return tca6408a_err_t This function is not implemented yet.
 * @note The return value is platform-dependent. For ESP32, ESP_OK means
 * success.
 * @warning Not implemented yet.
 */

// clang-format off
/**
 * @example
 * @code
 * tca6408a_dev_t dev;
 * tca6408a_err_t result = tca6408a_reset(&dev);
 * if (result == ESP_OK) {
 *     printf("Successfully reset the device.\n");
 * }
 * @endcode
 */
// clang-format on
tca6408a_err_t tca6408a_reset(const tca6408a_dev_t* dev_ptr)
{
#pragma message("tca6408a_reset is not implemented yet")
  return 0;
}

/* low level platform-dependent functions definitions */

/**
 * @brief Writes a new value to the specified register of the TCA6408A.
 *
 * @param dev_ptr Pointer to the device structure.
 * @param reg Register to write to.
 * @param new_val New value to write to the register.
 * @return esp_err_t Returns ESP_OK if the write operation was successful, or an
 * error code if the operation failed.
 * @note esp_err_t is specific to the ESP32 platform.
 */

// clang-format off
/**
 * @example
 * @code
 * tca6408a_dev_t dev;
 * uint8_t value = 0xFF;
 * tca6408a_err_t result = tca6408a_write_register(&dev, TCA6408A_OUTPUT_REG, value);
 * if (result == ESP_OK) {
 *     printf("Successfully wrote to the register.\n");
 * }
 * @endcode
 */
// clang-format on
tca6408a_err_t tca6408a_write_register(const tca6408a_dev_t* dev_ptr,
                                       tca6408a_reg reg, uint8_t new_val)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_ptr->address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, new_val, true);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(dev_ptr->i2c_bus, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * @brief Reads the value from the specified register of the TCA6408A.
 *
 * @param dev_ptr Pointer to the device structure.
 * @param reg Register to read from.
 * @param ret_val Pointer to store the read value.
 * @return esp_err_t Returns ESP_OK if the read operation was successful, or an
 * error code if the operation failed.
 * @note esp_err_t is specific to the ESP32 platform.
 */

// clang-format off
/**
 * @example
 * @code
 * tca6408a_dev_t dev;
 * uint8_t value;
 * tca6408a_err_t result = tca6408a_read_register(&dev, TCA6408A_INPUT_REG, &value);
 * if (result == ESP_OK) {
 *     printf("Read value: 0x%02X\n", value);
 * }
 * @endcode
 */
// clang-format on
tca6408a_err_t tca6408a_read_register(const tca6408a_dev_t* dev_ptr,
                                      tca6408a_reg reg, uint8_t* ret_val)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_ptr->address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);  // repeated start condition
  i2c_master_write_byte(cmd, (dev_ptr->address << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, ret_val, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(dev_ptr->i2c_bus, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * @brief Sets the specified bit to the given level in the specified register.
 *
 * @param dev_ptr Pointer to the device structure.
 * @param reg Register to modify.
 * @param nth_bit The bit number to set (0-7).
 * @param level The level to set the pin to (TCA6408A_LOW_LEVEL or
 * TCA6408A_HIGH_LEVEL).
 * @return tca6408a_err_t Returns ESP_OK if successful, or an error code if the
 * operation failed.
 * @note tca6408a_err_t is platform-dependent and could be esp_err_t or other
 * error codes depending on the platform.
 */

// clang-format off
/**
 * @example
 * @code
 * tca6408a_dev_t dev;
 * tca6408a_err_t result = tca6408a_set_bit_level(&dev, TCA6408A_OUTPUT_REG, 3, TCA6408A_HIGH_LEVEL);
 * if (result == ESP_OK) {
 *     printf("Successfully set pin 3 to high.\n");
 * }
 * @endcode
 */
// clang-format on
tca6408a_err_t tca6408a_set_bit_level(const tca6408a_dev_t* dev_ptr,
                                      tca6408a_reg reg, int nth_bit,
                                      tca6408a_pin_level level)
{
  uint8_t current_val;
  tca6408a_err_t ret;

  // read the current value of the register
  ret = tca6408a_read_register(dev_ptr, reg, &current_val);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // set or clear the nth bit
  if (level == TCA6408A_HIGH_LEVEL)
  {
    current_val |= (1 << nth_bit);
  }
  else
  {
    current_val &= ~(1 << nth_bit);
  }

  return tca6408a_write_register(dev_ptr, reg, current_val);
}

/**
 * @brief Sets the specified bits to the given level in the specified register.
 *
 * @param dev_ptr Pointer to the device structure.
 * @param reg Register to modify.
 * @param mask Mask indicating which bits to set (1 to modify, 0 to leave
 * unchanged).
 * @param level The level to set the bits to (TCA6408A_LOW_LEVEL or
 * TCA6408A_HIGH_LEVEL).
 * @return tca6408a_err_t Returns ESP_OK if successful, or an error code if the
 * operation failed.
 * @note tca6408a_err_t is platform-dependent and could be esp_err_t or other
 * error codes depending on the platform.
 */

// clang-format off
/**
 * @example
 * @code
 * tca6408a_dev_t dev;
 * tca6408a_err_t result = tca6408a_set_bits_level(&dev, TCA6408A_OUTPUT_REG, 0x0F, TCA6408A_HIGH_LEVEL);
 * if (result == ESP_OK) {
 *     printf("Successfully set lower 4 bits to high.\n");
 * }
 * @endcode
 */
// clang-format on
tca6408a_err_t tca6408a_set_bits_level(const tca6408a_dev_t* dev_ptr,
                                       tca6408a_reg reg, int mask,
                                       tca6408a_pin_level level)
{
  uint8_t current_val;
  tca6408a_err_t ret;

  // read the current value of the register
  ret = tca6408a_read_register(dev_ptr, reg, &current_val);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // set or clear the bits indicated by the mask
  if (level == TCA6408A_HIGH_LEVEL)
  {
    current_val |= mask;
  }
  else
  {
    current_val &= ~mask;
  }

  return tca6408a_write_register(dev_ptr, reg, current_val);
}

/**
 * @brief Sets the specified bits of the TCA6408A register to the given value.
 *
 * @param dev_ptr Pointer to the device structure.
 * @param reg Register to modify.
 * @param mask Mask indicating which bits to set (1 to modify, 0 to leave
 * unchanged).
 * @param value New value to set. Only the bits set to high in the mask will be
 * affected in the register.
 * @return tca6408a_err_t Returns ESP_OK if successful, or an error code if the
 * operation failed.
 * @note tca6408a_err_t is platform-dependent and could be esp_err_t or other
 * error codes depending on the platform.
 */

// clang-format off
/**
 * @example
 * @code
 * tca6408a_dev_t dev;
 * int mask = 0x0F;
 * int value = 0x05;
 * tca6408a_err_t result = tca6408a_set_bits(&dev, TCA6408A_OUTPUT_REG, mask, value);
 * if (result == ESP_OK) {
 *     printf("Successfully set lower 4 bits to 0x05.\n");
 * }
 * @endcode
 */
// clang-format on
tca6408a_err_t tca6408a_set_bits(const tca6408a_dev_t* dev_ptr,
                                 tca6408a_reg reg, int mask, int value)
{
  uint8_t current_val;
  tca6408a_err_t ret;

  // read the current value of the register
  ret = tca6408a_read_register(dev_ptr, reg, &current_val);
  if (ret != ESP_OK)
  {
    return ret;
  }

  // combine the current value with the new value, applying the mask to
  // determine which bits to change
  current_val = (current_val & (~mask)) | (value & mask);

  return tca6408a_write_register(dev_ptr, reg, current_val);
}