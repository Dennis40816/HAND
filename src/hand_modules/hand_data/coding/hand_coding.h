#pragma once
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

/* This file is for encode, decode and msg related */

#include "pb_encode.h"
#include "hand_data/proto/hand_data.pb.h"

#include "esp_timer.h"

/* public struct */
typedef struct hand_float_arr_arg_t
{
  float *fp; /* Pointer to the float array. */
  int count; /* Number of float data elements in the array. */
} hand_float_arr_arg_t;

typedef struct hand_timestamps_arr_arg_t
{
  int64_t *ts_p;
  int count;
} hand_timestamps_arr_arg_t;

typedef struct hand_data_msgs_arr_arg_t
{
  HandDataMsg** msgs_pp;
  int count; // msg number
} hand_data_msgs_arr_arg_t;

/* alias hand encode args */
typedef hand_float_arr_arg_t hand_vl53l1x_data_arg_t;

/* public API */

bool hand_encode_float_array(pb_ostream_t *stream, const pb_field_t *field,
                             void *const *arg);

/* TODO: decode_float_array */

bool hand_encode_timestamps_array(pb_ostream_t *stream, const pb_field_t *field,
                                  void *const *arg);

/* TODO: decode_timestamps_array */

/**
 * @brief 
 * 
 * @param stream 
 * @param field 
 * @param arg 
 * @return true 
 * @return false 
 */
bool hand_encode_data_msg_pointers_array(pb_ostream_t *stream,
                                         const pb_field_t *field,
                                         void *const *arg);

bool hand_overwrite_buf_bytes_count(uint8_t *buf, uint32_t bytes_count);

/* TODO: public alias, check good enough for using (improve readability)
 */
// __attribute__((alias("hand_encode_float_array"))) bool
// hand_encode_vl53l1x_data_array(pb_ostream_t *stream, const pb_field_t *field,
//                                void *const *arg);