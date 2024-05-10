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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef LIB_TCA6408A_H_
#define LIB_TCA6408A_H_

#include <stdint.h>

#include "driver/i2c.h"
#include "esp_err.h"

#define TCA6408A_DEFAULT_I2C_ADDRESS 0x20

// TCA6408A Register addresses
#define TCA6408A_INPUT_REG    0x00
#define TCA6408A_OUTPUT_REG   0x01
#define TCA6408A_POLARITY_REG 0x02
#define TCA6408A_CONFIG_REG   0x03

/* Config macros */
#define TCA6408A_CONFIG_OUTPUT (0)
#define TCA6408A_CONFIG_INPUT  (1)

#ifdef ESP_PLATFORM
typedef i2c_port_t i2c_port_type;
#else
typedef i2c_port_t int;
#endif

typedef struct {
  i2c_port_type i2c_port;
} tca6408a_t;

esp_err_t tca6408a_write_register(uint8_t reg, uint8_t value,
                                  const tca6408a_t* config);
esp_err_t tca6408a_read_register(uint8_t reg, uint8_t* value,
                                 const tca6408a_t* config);
esp_err_t tca6408a_set_input(uint8_t value, const tca6408a_t* config);
esp_err_t tca6408a_set_output(uint8_t value, const tca6408a_t* config);
esp_err_t tca6408a_set_low(uint8_t mask, const tca6408a_t* config);
esp_err_t tca6408a_set_high(uint8_t value, uint8_t mask, const tca6408a_t* config);

/* Debug */
void tca6408a_test_read(const tca6408a_t* config);

#endif