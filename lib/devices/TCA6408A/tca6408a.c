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

#ifdef LIB_USE_ESPIDF_PLATFORM
#pragma message("LIB_USE_ESPIDF_PLATFORM defined")
#else
#pragma message("LIB_USE_DUMMY_PLATFORM defined")
#endif

tca6408a_err_t tca6408a_set_pin_high(const tca6408a_dev_t* dev_ptr, int nth_pin)
{
  return tca6408a_set_bit_level(dev_ptr, TCA6408A_OUTPUT_REG, nth_pin,
                                TCA6408A_HIGH_LEVEL);
}
tca6408a_err_t tca6408a_set_pin_low(const tca6408a_dev_t* dev_ptr, int nth_pin)
{
  return tca6408a_set_bit_level(dev_ptr, TCA6408A_OUTPUT_REG, nth_pin,
                                TCA6408A_LOW_LEVEL);
}
tca6408a_err_t tca6408a_set_pin_output_mode(const tca6408a_dev_t* dev_ptr,
                                            int nth_pin)
{
  return tca6408a_set_bit_level(dev_ptr, TCA6408A_CONFIG_REG, nth_pin,
                                TCA6408A_IO_MODE_OUTPUT);
}
tca6408a_err_t tca6408a_set_pin_input_mode(const tca6408a_dev_t* dev_ptr,
                                           int nth_pin)
{
  return tca6408a_set_bit_level(dev_ptr, TCA6408A_CONFIG_REG, nth_pin,
                                TCA6408A_IO_MODE_INPUT);
}
tca6408a_err_t tca6408a_set_pin_polarity_invert(const tca6408a_dev_t* dev_ptr,
                                                int nth_pin)
{
  return tca6408a_set_bit_level(dev_ptr, TCA6408A_POLARITY_REG, nth_pin,
                                TCA6408A_HIGH_LEVEL);
}
tca6408a_err_t tca6408a_set_pin_polarity_normal(const tca6408a_dev_t* dev_ptr,
                                                int nth_pin)
{
  return tca6408a_set_bit_level(dev_ptr, TCA6408A_POLARITY_REG, nth_pin,
                                TCA6408A_LOW_LEVEL);
}