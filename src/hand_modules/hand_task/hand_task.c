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
#include "hand_global.h"
#include "hand_config.h"
#include "hand_task_priority.h"
#include "hand_task.h"
#include "hand_data/hand_data.h"
#include "hand_data/proto/hand_data.pb.h"
#include "hand_data/coding/hand_coding.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char* TAG = "HAND_TASK";

void hand_task_vl53l1x_from_queue_to_ppb(void* arg)
{
  hand_vl53l1x_data_element_t new_vl53l1x_data;
  hand_ppb_vl53l1x_data_t* const ppb_p = &hand_global_vl53l1x_ping_pong_buffer;
  while (1)
  {
    /* pop new_vl53l1x_data from queue */
    if (xQueueReceive(hand_global_vl53l1x_data_queue, &new_vl53l1x_data,
                      portMAX_DELAY) == pdTRUE)
    {
      xSemaphoreTake(hand_global_vl53l1x_ping_pong_mutex, portMAX_DELAY);

      /* get current buffer index */
      uint8_t buf_index = ppb_p->ping_pong_flag ? HAND_PPB_SET_BUFFER_INDEX
                                                : HAND_PPB_UNSET_BUFFER_INDEX;

      /* assign new value */
      ppb_p->data[buf_index].timestamps[ppb_p->current_index] =
          new_vl53l1x_data.timestamp;
      ppb_p->data[buf_index].data1[ppb_p->current_index] =
          new_vl53l1x_data.data1;
      ppb_p->data[buf_index].data2[ppb_p->current_index] =
          new_vl53l1x_data.data2;

      /* increase current_index */
      ++(ppb_p->current_index);

      /* current_index check */
      if (ppb_p->current_index >= HAND_SIZE_PPB_VL53L1X)
      {
        ppb_p->current_index = 0;
        ppb_p->ping_pong_flag = !ppb_p->ping_pong_flag;
      }

      xSemaphoreGive(hand_global_vl53l1x_ping_pong_mutex);
    }
  }
}

void hand_task_vl53l1x_send_data(void* arg)
{
  hand_ppb_vl53l1x_data_t* const ppb_p = &hand_global_vl53l1x_ping_pong_buffer;
  uint8_t buffer[HAND_SIZE_NANOPB_BUFFER_VL53L1X];

  hand_task_arg_vl53l1x_send_data_t* _arg =
      (hand_task_arg_vl53l1x_send_data_t*)arg;

  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // local var
    uint8_t send_buffer_index = 0;
    uint16_t send_data_index = 0;

    HandMsg hand_msg = HandMsg_init_zero;
    // Set direction and message type
    hand_msg.bytes_count = HAND_MSG_BYTES_COUNT_PLACEHOLDER_VALUE;
    hand_msg.direction = HandMsgDirection_FROM_HAND;
    hand_msg.msg_type = HandMainMsgType_DATA;
    hand_msg.chip_type = HandChipType_VL53L1X;

    // Set content
    hand_msg.which_content = HandMsg_data_wrapper_tag;

    // Take the mutex before accessing shared resources, may blocked here
    if (xSemaphoreTake(hand_global_vl53l1x_ping_pong_mutex, portMAX_DELAY) ==
        pdTRUE)
    {
      // Determine which buffer to send
      send_buffer_index = ppb_p->ping_pong_flag ? 1 : 0;
      // Index for storing the next data item
      send_data_index = ppb_p->current_index;
      // For to swap buffers
      ppb_p->current_index = 0;
      ppb_p->ping_pong_flag = !ppb_p->ping_pong_flag;

      // Release the mutex immediately after accessing shared data
      xSemaphoreGive(hand_global_vl53l1x_ping_pong_mutex);
    }

    if (send_data_index != 0)
    {
      ESP_LOGI(TAG, "send data index is %d", send_data_index);
      /* format args */
      hand_timestamps_arr_arg_t timestamps_arg = {
          .ts_p = ppb_p->data[send_buffer_index].timestamps,
          .count = send_data_index  // data number
      };

      hand_vl53l1x_data_arg_t vl53l1x_sensor1_data_arg = {
          .fp = ppb_p->data[send_buffer_index].data1,
          .count = send_data_index  // data number
      };

      hand_vl53l1x_data_arg_t vl53l1x_sensor2_data_arg = {
          .fp = ppb_p->data[send_buffer_index].data2,
          .count = send_data_index  // data number
      };

      // Fill first sensor data
      HandDataMsg data_msg1 = HandDataMsg_init_zero;
      data_msg1.source = HandChipInstance_VL53L1X_SENSOR1;
      data_msg1.data_type = HandDataType_FLOAT;
      data_msg1.data_count = send_data_index;
      // use timestamps instead of timestamp
      data_msg1.has_timestamp = false;
      data_msg1.timestamps.funcs.encode = hand_encode_timestamps_array;
      data_msg1.timestamps.arg = &timestamps_arg;
      data_msg1.data.funcs.encode = hand_encode_float_array;
      data_msg1.data.arg = &vl53l1x_sensor1_data_arg;

      // Fill second sensor data
      HandDataMsg data_msg2 = HandDataMsg_init_zero;
      data_msg2.source = HandChipInstance_VL53L1X_SENSOR2;
      data_msg2.data_type = HandDataType_FLOAT;
      data_msg2.data_count = send_data_index;
      data_msg2.has_timestamp = false;
      data_msg2.timestamps.funcs.encode = hand_encode_timestamps_array;
      data_msg2.timestamps.arg = &timestamps_arg;
      data_msg2.data.funcs.encode = hand_encode_float_array;
      data_msg2.data.arg = &vl53l1x_sensor2_data_arg;

      HandDataMsg* data_msgs[HAND_DEV_MAX_NUM_VL53L1X] = {&data_msg1,
                                                          &data_msg2};

      // create msg arg
      hand_data_msgs_arr_arg_t msg_arg = {
          .msgs_pp = data_msgs,
          .count = HAND_DEV_MAX_NUM_VL53L1X  // total 2 vl53l1x data msgs
      };

      hand_msg.content.data_wrapper.data_msgs.funcs.encode =
          hand_encode_data_msg_pointers_array;
      hand_msg.content.data_wrapper.data_msgs.arg = &msg_arg; // TODO: check this correct or not
    }
  }
}