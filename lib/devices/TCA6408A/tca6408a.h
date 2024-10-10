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

#ifndef LIB_TCA6408A_TCA6408A_H_
#define LIB_TCA6408A_TCA6408A_H_

#include <stdint.h>

#ifdef LIB_USE_ESPIDF_PLATFORM
#include "platform/espidf/tca6408a_espidf.h"
#else
#include "platform/dummy/tca6408a_dummy.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* tca6408a struct definition */
typedef struct tca6408a_dev_t tca6408a_dev_t;
typedef struct tca6408a_reg_info_t tca6408a_reg_info_t;
typedef struct tca6408a_i2c_cmd_t tca6408a_i2c_cmd_t;  // for callback

struct tca6408a_dev_t
{
  uint8_t address; /* should be 0x20 or 0x21 */
  tca6408a_i2c_t i2c_bus;
  /* TODO: add address_gpio_pin, change_address() */
  /* TODO: add reset_gpio_pin */
};

struct tca6408a_reg_info_t
{
  uint8_t input_reg;
  uint8_t output_reg;
  uint8_t polarity_reg;
  uint8_t config_reg;
};

/* register internal address enum */
typedef enum
{
  TCA6408A_INPUT_REG,
  TCA6408A_OUTPUT_REG,
  TCA6408A_POLARITY_REG,
  TCA6408A_CONFIG_REG,
  // END OF REGISTERS
  TCA6408A_NUM_REGS
} tca6408a_reg;

/* input output mode enum */
typedef enum
{
  TCA6408A_IO_MODE_OUTPUT,
  TCA6408A_IO_MODE_INPUT
} tca6408a_io_mode;

typedef enum
{
  TCA6408A_LOW_LEVEL,
  TCA6408A_HIGH_LEVEL
} tca6408a_pin_level;

/* high level platform-agnostic functions declarations (recommended) */
tca6408a_err_t tca6408a_set_pin_high(const tca6408a_dev_t* dev_ptr,
                                     int nth_pin);
tca6408a_err_t tca6408a_set_pin_low(const tca6408a_dev_t* dev_ptr, int nth_pin);
tca6408a_err_t tca6408a_set_pin_output_mode(const tca6408a_dev_t* dev_ptr,
                                            int nth_pin);
tca6408a_err_t tca6408a_set_pin_input_mode(const tca6408a_dev_t* dev_ptr,
                                           int nth_pin);
tca6408a_err_t tca6408a_set_pin_polarity_invert(const tca6408a_dev_t* dev_ptr,
                                                int nth_pin);
tca6408a_err_t tca6408a_set_pin_polarity_normal(const tca6408a_dev_t* dev_ptr,
                                                int nth_pin);

/* high level platform-dependent functions declarations (recommended) */
tca6408a_err_t tca6408a_reg_info_to_str(const tca6408a_reg_info_t* info,
                                        char* str, uint32_t max_len);
tca6408a_err_t tca6408a_read_all(const tca6408a_dev_t* dev_ptr,
                                 tca6408a_reg_info_t* info);
tca6408a_err_t tca6408a_reset(const tca6408a_dev_t* dev_ptr);  // TODO: not impl

/* low level platform-dependent functions declarations */
tca6408a_err_t tca6408a_set_bit_level(const tca6408a_dev_t* dev_ptr,
                                      tca6408a_reg reg, int nth_bit,
                                      tca6408a_pin_level level);
tca6408a_err_t tca6408a_set_bits_level(const tca6408a_dev_t* dev_ptr,
                                       tca6408a_reg reg, int mask,
                                       tca6408a_pin_level level);
tca6408a_err_t tca6408a_set_bits(const tca6408a_dev_t* dev_ptr,
                                       tca6408a_reg reg, int mask,
                                       int value);
tca6408a_err_t tca6408a_write_register(const tca6408a_dev_t* dev_ptr,
                                       tca6408a_reg reg, uint8_t new_val);
tca6408a_err_t tca6408a_read_register(const tca6408a_dev_t* dev_ptr,
                                      tca6408a_reg reg, uint8_t* ret_val);

#ifdef __cplusplus
}
#endif
#endif