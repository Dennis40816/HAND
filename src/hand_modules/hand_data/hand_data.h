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

/**
 * @brief Defines the length and offset values for the `bytes_count` field in
 * HAND_MSG.
 *
 * The `bytes_count` field length is defined as 1 byte. This is consistent with
 * the general protobuf rule where all fields are represented by 1 byte in the
 * message. The `bytes_count` field is always the first field in a HAND_MSG
 * structure and is of type `fixed32`, which means it occupies a fixed size of 4
 * bytes in the message. The value of this field is located at the 1st to 4th
 * bytes of the message when it is non-zero.
 *
 * The purpose of `HAND_MSG_BYTES_COUNT_PLACEHOLDER_VALUE` is to provide a
 * placeholder value for this field. This ensures that when the `bytes_count`
 * value is not required or is zero, a placeholder value of 1 is used to
 * maintain the structure's integrity.
 */
// being the first field, so the offset is 0
#define HAND_MSG_BYTES_COUNT_FIELD_OFFSET (0)
#define HAND_MSG_BYTES_COUNT_FIELD_LENGTH (1)
#define HAND_MSG_BYTES_COUNT_VALUE_OFFSET \
  (HAND_MSG_BYTES_COUNT_FIELD_OFFSET + HAND_MSG_BYTES_COUNT_FIELD_LENGTH)
// fixed32 is 4 bytes len
#define HAND_MSG_BYTES_COUNT_VALUE_LENGTH      (4)
#define HAND_MSG_BYTES_COUNT_PLACEHOLDER_VALUE (1)

/* public struct */

typedef struct hand_vl53l1x_data_element_t
{
  int64_t timestamp;
  /* TODO: use array? */
  float data1;  // VL53L1X_1
  float data2;  // VL53L1X_2
} hand_vl53l1x_data_element_t;

typedef struct hand_vl53l1x_data_t
{
  int64_t timestamps[HAND_SIZE_PPB_VL53L1X];
  float data1[HAND_SIZE_PPB_VL53L1X];
  float data2[HAND_SIZE_PPB_VL53L1X];
} hand_vl53l1x_data_t;

typedef struct hand_chx01_simple_data_unit_t
{
  uint16_t sample_num;
  uint16_t amp;
  float range;
} hand_chx01_simple_data_unit_t;

/* for queue */
typedef struct hand_chx01_group_data_element_t
{
  int64_t timestamp;
  hand_chx01_simple_data_unit_t data[HAND_DEV_MAX_NUM_CH101];
} hand_chx01_group_data_element_t;

/* for nanopb */
typedef struct hand_chx01_data_t
{
  int64_t timestamps[HAND_SIZE_PPB_CH101];
  hand_chx01_simple_data_unit_t data[HAND_DEV_MAX_NUM_CH101]
                                    [HAND_SIZE_PPB_CH101];
} hand_chx01_data_t;

/* ping pong buffer struct (PPB) */
#define HAND_PPB_UNSET_BUFFER_INDEX (0)
#define HAND_PPB_SET_BUFFER_INDEX   (1)

// VL53L1X related

#define HAND_PPB_VL53L1X_BUFFER_NUM (2)
typedef struct hand_ppb_vl53l1x_data_t
{
  bool ping_pong_flag;
  uint16_t current_index;
  hand_vl53l1x_data_t data[HAND_PPB_VL53L1X_BUFFER_NUM];
} hand_ppb_vl53l1x_data_t;

// CH101 related

#define HAND_PPB_CH101_BUFFER_NUM (2)
typedef struct hand_ppb_ch101_data_t
{
  bool ping_pong_flag;
  uint16_t current_index;
  hand_chx01_data_t data[HAND_PPB_CH101_BUFFER_NUM];
} hand_ppb_ch101_data_t;