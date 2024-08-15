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

#include "stdint.h"
#include "hand_config.h"

/* public define */
// This is a placeholder value (can not be 0) for hand msg, the bytes_count
// field will be replaced by hand_overwrite_buf_bytes_count()
#define HAND_MSG_BYTES_COUNT_PLACEHOLDER_VALUE (1)

/* public struct */
typedef struct hand_vl53l1x_data_element_t
{
  int64_t timestamp;
  /* TODO: use array? */
  float data1;  // VL53L1X_1
  float data2;  // VL53L1X_2
} hand_vl53l1x_data_element_t;

typedef struct hand_chx01_data_element_t
{
  int64_t timestamp;
  /* TODO: use array? */
  float data1;  // CH101_1
  float data2;  // CH101_2
  float data3;  // CH101_3
  float data4;  // CH101_4
} hand_chx01_data_element_t;

typedef struct hand_vl53l1x_data_t
{
  int64_t timestamps[HAND_SIZE_PPB_VL53L1X];
  float data1[HAND_SIZE_PPB_VL53L1X];
  float data2[HAND_SIZE_PPB_VL53L1X];
} hand_vl53l1x_data_t;

typedef struct hand_chx01_data_t
{
  int64_t timestamps[HAND_SIZE_PPB_CH101];
  float data1[HAND_SIZE_PPB_CH101];
  float data2[HAND_SIZE_PPB_CH101];
  float data3[HAND_SIZE_PPB_CH101];
  float data4[HAND_SIZE_PPB_CH101];
} hand_chx01_data_t;

/* ping pong buffer struct (PPB) */
#define HAND_PPB_UNSET_BUFFER_INDEX (0)
#define HAND_PPB_SET_BUFFER_INDEX   (1)

#define HAND_PPB_VL53L1X_BUFFER_NUM (2)
typedef struct hand_ppb_vl53l1x_data_t
{
  bool ping_pong_flag;
  uint16_t current_index;
  hand_vl53l1x_data_t data[HAND_PPB_VL53L1X_BUFFER_NUM];
} hand_ppb_vl53l1x_data_t;