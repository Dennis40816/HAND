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
#include "tca6408a_dummy.h"

tca6408a_err_t tca6408a_reg_info_to_str(const tca6408a_reg_info_t* info,
                                        char* str, uint32_t max_len)
{
    return 0;
}

tca6408a_err_t tca6408a_read_all(const tca6408a_dev_t* dev_ptr,
                                 tca6408a_reg_info_t* info)
{
    return 0;
}

tca6408a_err_t tca6408a_reset(const tca6408a_dev_t* dev_ptr)
{
    return 0;
}

tca6408a_err_t tca6408a_write_register(const tca6408a_dev_t* dev_ptr,
                                       tca6408a_reg reg, uint8_t new_val)
{
    return 0;
}

tca6408a_err_t tca6408a_read_register(const tca6408a_dev_t* dev_ptr,
                                      tca6408a_reg reg, uint8_t* ret_val)
{
    return 0;
}

tca6408a_err_t tca6408a_set_bit_level(const tca6408a_dev_t* dev_ptr,
                                      tca6408a_reg reg, int nth_bit,
                                      tca6408a_pin_level level)
{
    return 0;
}

tca6408a_err_t tca6408a_set_bits_level(const tca6408a_dev_t* dev_ptr,
                                       tca6408a_reg reg, int mask,
                                       tca6408a_pin_level level)
{
    return 0;
}

tca6408a_err_t tca6408a_set_bits(const tca6408a_dev_t* dev_ptr,
                                 tca6408a_reg reg, int mask, int value)
{
    return 0;
}