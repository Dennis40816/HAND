#include <tca6408a.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * Write a value to a register
 */
esp_err_t tca6408a_write_register(uint8_t reg, uint8_t value,
                                  const tca6408a_t* config) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  /* TODO: can be improved by i2c_master_write */
  i2c_master_write_byte(
      cmd, (TCA6408A_DEFAULT_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, value, true);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(config->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * Read a value from a register
 */
esp_err_t tca6408a_read_register(uint8_t reg, uint8_t* value,
                                 const tca6408a_t* config) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  /* TODO: can be improved by i2c_master_write */
  i2c_master_write_byte(
      cmd, (TCA6408A_DEFAULT_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);  // repeated start condition
  i2c_master_write_byte(
      cmd, (TCA6408A_DEFAULT_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, value, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(config->i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * @brief Configures specified pins of the TCA6408A as output pins. This
 * function modifies the configuration register to set pins as outputs based on
 * the provided mask. Note that setting a bit to 1 in the mask will NOT set it
 * to input; it will only clear bits to set them as outputs.
 *
 * @param value A mask indicating which bits to set as output. Bits set to 1 in
 * the mask will be configured as output pins.
 * @param config Pointer to the TCA6408A configuration structure.
 * @example tca6408a_set_output(0b00000101, &config); // Sets pins P0 and P2 as
 * outputs.
 * @return esp_err_t Returns ESP_OK if the operation was successful, or an error
 * code if the operation failed.
 */
esp_err_t tca6408a_set_output(uint8_t value, const tca6408a_t* config) {
  uint8_t current_val;
  tca6408a_read_register(TCA6408A_CONFIG_REG, &current_val, config);

  uint8_t final_val = current_val & (~value);
  return tca6408a_write_register(TCA6408A_CONFIG_REG, final_val, config);
}

/**
 * @brief Configures specified pins of the TCA6408A as input pins. This function
 * modifies the configuration register to set pins as inputs based on the
 * provided mask. Setting a bit to 1 in the mask will configure that pin as an
 * input.
 *
 * @param value A mask indicating which bits to set as input. Bits set to 1 in
 * the mask will be configured as input pins.
 * @param config Pointer to the TCA6408A configuration structure.
 * @example tca6408a_set_input(0b00000110, &config); // Sets pins P1 and P2 as
 * inputs.
 * @return esp_err_t Returns ESP_OK if the operation was successful, or an error
 * code if the operation failed.
 */
esp_err_t tca6408a_set_input(uint8_t value, const tca6408a_t* config) {
  uint8_t current_val;
  tca6408a_read_register(TCA6408A_CONFIG_REG, &current_val, config);

  uint8_t final_val = current_val | value;
  return tca6408a_write_register(TCA6408A_CONFIG_REG, final_val, config);
}

/**
 * @brief Sets specified bits of the TCA6408A output register to high, according
 * to the mask and value provided. This function first reads the current value
 * of the output register, then modifies it based on the mask and value
 * parameters before writing it back.
 *
 * @param value New value to set. Only the bits set to high in the mask will be
 * affected in the output register.
 * @param mask Set the bit you want to modify to high, otherwise keep it low.
 * Acts as a filter to decide which bits in the 'value' parameter should affect
 * the output register.
 * @example tca6408a_set_high(1 << 7, 1 << 7, &config), sets P7 to high. This
 * example sets the 7th bit of the TCA6408A output register to high without
 * altering other bits.
 * @return esp_err_t Returns ESP_OK if the write operation was successful, or an
 * error code if the operation failed.
 */
esp_err_t tca6408a_set_high(uint8_t value, uint8_t mask, const tca6408a_t* config) {
  uint8_t current_val;
  tca6408a_read_register(TCA6408A_OUTPUT_REG, &current_val, config);

  // Combine the current value with the new value, applying the mask to
  // determine which bits to change
  uint8_t final_val = (current_val & (~mask)) | (value & mask);
  return tca6408a_write_register(TCA6408A_OUTPUT_REG, final_val, config);
}

/**
 * Sets specific bits of the TCA6408A to low level.
 * @param mask Set the bit you want to modify to high, others keep low.
 * @param config Pointer to the TCA6408A configuration structure.
 * @example tca6408a_set_low(1 << 7, &config), sets P7 to low level
 * @return esp_err_t
 */
esp_err_t tca6408a_set_low(uint8_t mask, const tca6408a_t* config) {
  uint8_t current_val;
  tca6408a_read_register(TCA6408A_OUTPUT_REG, &current_val, config);

  // Clear specified bits using the mask
  uint8_t final_val = current_val & (~mask);
  return tca6408a_write_register(TCA6408A_OUTPUT_REG, final_val, config);
}

/*
set 0 is output
*/
// void tca6408a_init(void) {
//     // Configure P7 and P6 as outputs, others as inputs
//     tca6408a_write_register(TCA6408A_CONFIG_REG, (uint8_t)(~(1 << 7 | 1 << 6
//     | 1 << 5)));
//     // Set P7 high, others remain in their current state (P6 low by default)
//     tca6408a_write_register(TCA6408A_OUTPUT_REG, 1 << 7 | 1 << 5);
// }

/**
 * Test read function
 */
void tca6408a_test_read(const tca6408a_t* config) {
    for (uint8_t target = 0x0; target <= TCA6408A_CONFIG_REG; ++target)
    {
        uint8_t data = 0;
        if (tca6408a_read_register(target, &data, config) == ESP_OK) {
            ESP_LOGI("tca6408a_lib", "Read data: %d%d%d%d %d%d%d%d. @%x",
            (data & 0x80) >> 7, (data & 0x40) >> 6, (data & 0x20) >> 5, (data
            & 0x10) >> 4, (data & 0x08) >> 3, (data & 0x04) >> 2, (data &
            0x02) >> 1, (data & 0x01), target);
        } else {
            ESP_LOGI("tca6408a_lib", "Failed to read from TCA6408A\n");
        }
    }
}