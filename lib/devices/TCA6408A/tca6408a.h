#ifndef LIB_TCA6408A_H_
#define LIB_TCA6408A_H_

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c.h"

#define TCA6408A_DEFAULT_I2C_ADDRESS   0x20

// TCA6408A Register addresses
#define TCA6408A_INPUT_REG     0x00
#define TCA6408A_OUTPUT_REG    0x01
#define TCA6408A_POLARITY_REG  0x02
#define TCA6408A_CONFIG_REG    0x03

#define TCA6408A_DEFAULT_I2C   I2C_NUM_0

esp_err_t tca6408a_write_register(uint8_t reg, uint8_t value);
esp_err_t tca6408a_read_register(uint8_t reg, uint8_t *value);
esp_err_t tca6408a_set_output(uint8_t value);
esp_err_t tca6408a_set_high(uint8_t value, uint8_t mask);
esp_err_t tca6408a_set_input(uint8_t value);

// void tca6408a_init(void);
// void tca6408a_test_read(void);

#endif