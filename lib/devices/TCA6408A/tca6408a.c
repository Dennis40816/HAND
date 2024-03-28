#include <tca6408a.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * Write a value to a register
 */
esp_err_t tca6408a_write_register(uint8_t reg, uint8_t value, tca6408a_t* config) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(
      cmd, (TCA6408A_DEFAULT_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_write_byte(cmd, value, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(config->i2c_port, cmd,
                                       1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * Read a value from a register
 */
esp_err_t tca6408a_read_register(uint8_t reg, uint8_t *value, tca6408a_t* config) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(
      cmd, (TCA6408A_DEFAULT_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg, true);
  i2c_master_start(cmd);  // repeated start condition
  i2c_master_write_byte(
      cmd, (TCA6408A_DEFAULT_I2C_ADDRESS << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, value, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(config->i2c_port, cmd,
                                       1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

/**
 * @brief Will not set to input even you input 1 in that bit!!
 *
 * @param value a mask that you want to change it to output
 * @return esp_err_t
 */
esp_err_t tca6408a_set_output(uint8_t value, tca6408a_t* config) {
  uint8_t current_val;
  tca6408a_read_register(TCA6408A_CONFIG_REG, &current_val, config);

  uint8_t final_val = current_val & (~value);
  return tca6408a_write_register(TCA6408A_CONFIG_REG, final_val, config);
}

/**
 * @brief
 *
 * @param value a mask that you want to change it to output
 * @return esp_err_t
 */
esp_err_t tca6408a_set_input(uint8_t value, tca6408a_t* config) {
  uint8_t current_val;
  tca6408a_read_register(TCA6408A_CONFIG_REG, &current_val, config);

  uint8_t final_val = current_val | value;
  return tca6408a_write_register(TCA6408A_CONFIG_REG, final_val, config);
}

/**
 * @brief
 *
 * @param value new value, will be
 * @param mask set the bit you want to modify to high, otherwise keep it low.
 * @example tca6408a_set_high(1 << 7, 1 << 7), set p7 to high
 * @return esp_err_t
 */
esp_err_t tca6408a_set_high(uint8_t value, uint8_t mask, tca6408a_t* config) {
  uint8_t current_val;
  tca6408a_read_register(TCA6408A_OUTPUT_REG, &current_val, config);

  // and reverse mask, and value -> keep not
  uint8_t final_val = (current_val & (~mask)) | (value & mask);
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
// void tca6408a_test_read(void) {
//     for (uint8_t target = 0x0; target <= TCA6408A_CONFIG_REG; ++target)
//     {
//         uint8_t data = 0;
//         if (tca6408a_read_register(target, &data) == ESP_OK) {
//             ESP_LOGI("tca6408a_lib", "Read data: %d%d%d%d %d%d%d%d. @%x",
//             (data & 0x80) >> 7, (data & 0x40) >> 6, (data & 0x20) >> 5, (data
//             & 0x10) >> 4, (data & 0x08) >> 3, (data & 0x04) >> 2, (data &
//             0x02) >> 1, (data & 0x01), target);
//         } else {
//             ESP_LOGE("tca6408a_lib", "Failed to read from TCA6408A\n");
//         }
//     }
// }