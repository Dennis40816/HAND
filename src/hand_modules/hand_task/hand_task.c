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

#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// for for loop using
#define HAND_MSG_SOURCE_CH101_BASE (HandChipInstance_CH101_SENSOR1)

static const char* TAG = "HAND_TASK";

void hand_task_vl53l1x_collect_data(void* __attribute__((unused)) arg)
{
  VL53L1_Dev_t* vl53l1x_dev_p = hand_global_devs_handle.vl53l1x_dev;
  VL53L1_Error ret;
  EventBits_t event_bits;

  /* start measurement procedure on vl53l1x group  */
  for (uint8_t dev_index = 0; dev_index < HAND_DEV_MAX_NUM_VL53L1X; ++dev_index)
  {
    ret = VL53L1_StartMeasurement(&(vl53l1x_dev_p[dev_index]));
    if (ret == VL53L1_ERROR_NONE)
    {
      /* start successfully */
      ESP_LOGI(TAG, "VL53L1X_{%d} started measurement successfully",
               dev_index + 1);
    }
    else
    {
      ESP_LOGE(
          TAG,
          "VL53L1X_{%d} FAILED to start measurement. VL53L1 Error code: {%d} ",
          dev_index + 1, ret);
    }

    /* TODO: record start measurement states and only collect data from active
     * device */
  }

  /* TODO: add a stop flag in arg */
  /* collect data permanently */
  while (1)
  {
    /* TODO: Allow the task to continue after waiting for n milliseconds when
   only one flag is set. This ensures that if one VL53L1X sensor fails, the
   other can still operate. */
    event_bits = xEventGroupWaitBits(
        hand_global_vl53l1x_event_group,
        HAND_EG_VL53L1X_1_DATA_READY_BIT | HAND_EG_VL53L1X_2_DATA_READY_BIT,
        pdTRUE,  // clear wait bit when exit
        pdTRUE,  // fire only at all bits are set
        portMAX_DELAY);

    int64_t timestamp = esp_timer_get_time();

    /* init new vl53l1x data element */
    hand_vl53l1x_data_element_t new_vl53l1x_data = {0};
    new_vl53l1x_data.timestamp = timestamp;

    /* CRITICAL REGION: call hardware related function */
    /* TODO: make it a loop (like ) */
    /* pseudo code
       for (int i = 0, int v = 0; v < MAX_SHIFT_NUM; ++i, v <<= 1)
       {
          if (active_dev[i] == true)
          {
            // do measurement by &(vl53l1x_dev_p[i])
          }
       }
     */
    if (event_bits & HAND_EG_VL53L1X_1_DATA_READY_BIT)
    {
      VL53L1_RangingMeasurementData_t range_data;
      VL53L1_Error status =
          VL53L1_GetRangingMeasurementData(&(vl53l1x_dev_p[0]), &range_data);

      if (status == VL53L1_ERROR_NONE)
      {
        ESP_LOGV(TAG, "VL53L1X_1: %3.1f (cm)",
                 range_data.RangeMilliMeter / 10.0);
        new_vl53l1x_data.data1 = range_data.RangeMilliMeter / 10.0;
      }
      else
      {
        /* TODO: error handle (set failure bit) */
      }

      status = VL53L1_ClearInterruptAndStartMeasurement(&(vl53l1x_dev_p[0]));

      if (status != VL53L1_ERROR_NONE)
      {
        ESP_LOGW(TAG,
                 "Status of VL53L1X_{%d} in "
                 "`VL53L1_ClearInterruptAndStartMeasurement` is: {%d}",
                 1, status);
        /* TODO: error handle (set failure bit) */
      }
    }

    if (event_bits & HAND_EG_VL53L1X_2_DATA_READY_BIT)
    {
      VL53L1_RangingMeasurementData_t range_data;
      VL53L1_Error status =
          VL53L1_GetRangingMeasurementData(&(vl53l1x_dev_p[1]), &range_data);

      if (status == VL53L1_ERROR_NONE)
      {
        ESP_LOGV(TAG, "VL53L1X_2: %3.1f (cm)",
                 range_data.RangeMilliMeter / 10.0);
        new_vl53l1x_data.data2 = range_data.RangeMilliMeter / 10.0;
      }
      else
      {
        /* TODO: error handle (set failure bit) */
      }

      status = VL53L1_ClearInterruptAndStartMeasurement(&(vl53l1x_dev_p[1]));

      if (status != VL53L1_ERROR_NONE)
      {
        ESP_LOGW(TAG,
                 "Status of VL53L1X_{%d} in "
                 "`VL53L1_ClearInterruptAndStartMeasurement` is: {%d}",
                 2, status);
        /* TODO: error handle (set failure bit) */
      }
    }

    /* TODO: check that using portMAX_DELAY is appropriate */
    xQueueSend(hand_global_vl53l1x_data_queue, &new_vl53l1x_data,
               pdMS_TO_TICKS(HAND_MS_VL53L1X_QUEUE_MAX_DELAY));

    /* TODO: check other bit is set or not -> error handle here (activate_dev[i]
     * = false...) */
  }
}

void hand_task_vl53l1x_from_queue_to_ppb(void* __attribute__((unused)) arg)
{
  hand_vl53l1x_data_element_t new_vl53l1x_data;
  volatile hand_ppb_vl53l1x_data_t* const ppb_p =
      &hand_global_vl53l1x_ping_pong_buffer;

  /* parse task arg (currently, remain unused))*/

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
  volatile hand_ppb_vl53l1x_data_t* const ppb_p =
      &hand_global_vl53l1x_ping_pong_buffer;
  uint8_t buffer[HAND_SIZE_NANOPB_BUFFER_VL53L1X] = {0};

  hand_task_arg_vl53l1x_send_data_t* task_arg_p =
      (hand_task_arg_vl53l1x_send_data_t*)arg;

  /* parse task arg */
  int client_socket = *task_arg_p;

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
      send_buffer_index = ppb_p->ping_pong_flag ? HAND_PPB_SET_BUFFER_INDEX
                                                : HAND_PPB_UNSET_BUFFER_INDEX;
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
      ESP_LOGD(TAG, "vl53l1x send data index is %d", send_data_index);
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
      hand_msg.content.data_wrapper.data_msgs.arg =
          &msg_arg;  // TODO: check this correct or not

      /* start to encode */
      int64_t start_time = esp_timer_get_time();

      /* prepare ostream */
      pb_ostream_t stream =
          pb_ostream_from_buffer(buffer, HAND_SIZE_NANOPB_BUFFER_VL53L1X);

      /* encode */
      if (!pb_encode(&stream, HandMsg_fields, &hand_msg))
      {
        ESP_LOGE(TAG, "VL53L1X msgs encoding failed: %s",
                 PB_GET_ERROR(&stream));
        // XXX: may cause forever loop
        continue;
      }

      int64_t end_time = esp_timer_get_time();
      int64_t encode_duration = end_time - start_time;
      ESP_LOGD(TAG,
               "VL53L1X message encoded successfully, size: %zu bytes, time: "
               "%lld us",
               stream.bytes_written, encode_duration);

      // Overwrite buffer[1:4] by the function
      hand_overwrite_buf_bytes_count(buffer, stream.bytes_written);

      // Timing start - sending data
      start_time = esp_timer_get_time();

      int ret = send(client_socket, buffer, stream.bytes_written, 0);
      if (ret < 0)
      {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
      }
      else
      {
        ESP_LOGV(TAG, "Message sent successfully");
      }

      end_time = esp_timer_get_time();
      int64_t transmit_duration = end_time - start_time;
      ESP_LOGD(TAG, "Data transmitted successfully, time: %lld us",
               transmit_duration);
    }
    // Delay until the next cycle
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(HAND_MS_VL53L1X_SEND_DATA));
  }
}

static void _hand_ch101_handle_data_ready(ch_group_t* grp_ptr)
{
  hand_chx01_group_data_element_t new_ch101_data;
  uint8_t dev_num;
  int num_samples = 0;

  new_ch101_data.timestamp = esp_timer_get_time();

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
  {
    uint32_t range = 0;
    ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    if (ch_sensor_is_connected(dev_ptr))
    {
      /* Get measurement results from each connected sensor
       *   For sensor in transmit/receive mode, report one-way echo
       *   distance,  For sensor(s) in receive-only mode, report direct
       *   one-way distance from transmitting sensor
       */

      if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY)
      {
        range = ch_get_range(dev_ptr, CH_RANGE_DIRECT);
      }
      else
      {
        range = ch_get_range(dev_ptr, CH_RANGE_ECHO_ONE_WAY);
      }

      if (range == CH_NO_TARGET)
      {
        new_ch101_data.data[dev_num].range = NAN;
        new_ch101_data.data[dev_num].amp = 0;

        // DEBUG only
        ESP_LOGD(TAG, "Port {%d} target not found", dev_num);
      }
      else
      {
        new_ch101_data.data[dev_num].range = range / 32.0f;
        new_ch101_data.data[dev_num].amp = ch_get_amplitude(dev_ptr);
        ESP_LOGD(TAG, "Port {%d}:  Range: %0.1f mm  Amp: %u", dev_num,
                 new_ch101_data.data[dev_num].range,
                 new_ch101_data.data[dev_num].amp);
      }

      num_samples = ch_get_num_samples(dev_ptr);
      new_ch101_data.data[dev_num].sample_num = num_samples;

      /* TODO: get amp data  */

      /* TODO: get IQ data */
    }
  }
  ESP_LOGI(TAG, "%.3f, %.3f, %.3f, %.3f", new_ch101_data.data[0].range,
           new_ch101_data.data[1].range, new_ch101_data.data[2].range,
           new_ch101_data.data[3].range);
  /* push to queue */
  xQueueSend(hand_global_ch101_data_queue, &new_ch101_data,
             pdMS_TO_TICKS(HAND_MS_CH101_QUEUE_MAX_DELAY));
}

void hand_task_ch101_collect_data(void* __attribute__((unused)) arg)
{
  /* start periodic timer */
  chbsp_periodic_timer_init(HAND_MS_CH101_DEFAULT_MEASURE_PERIOD,
                            hand_cb_ch101_periodic_timer);

  /* Enable interrupt and start timer to trigger sensor sampling */
  chbsp_periodic_timer_irq_enable();
  chbsp_periodic_timer_start();

  ESP_LOGI(TAG, "CH101 measurement is starting!");

  while (1)
  {
    /* wait for event, no ret for only one bit is waited */
    xEventGroupWaitBits(hand_global_ch101_event_group,
                        HAND_EG_CH101_ALL_ACTIVE_DEV_DATA_READY_BIT,
                        true,  // clear on exit
                        true, portMAX_DELAY);

    _hand_ch101_handle_data_ready(&hand_global_devs_handle.ch101_group);
  }
}

void hand_task_ch101_from_queue_to_ppb(void* __attribute__((unused)) arg)
{
  hand_chx01_group_data_element_t new_ch101_data;
  volatile hand_ppb_ch101_data_t* const ppb_p =
      &hand_global_ch101_ping_pong_buffer;

  /* parse task arg (currently, remain unused))*/

  while (1)
  {
    /* pop new_vl53l1x_data from queue */
    if (xQueueReceive(hand_global_ch101_data_queue, &new_ch101_data,
                      portMAX_DELAY) == pdTRUE)
    {
      xSemaphoreTake(hand_global_ch101_ping_pong_mutex, portMAX_DELAY);

      /* get current buffer index */
      uint8_t buf_index = ppb_p->ping_pong_flag ? HAND_PPB_SET_BUFFER_INDEX
                                                : HAND_PPB_UNSET_BUFFER_INDEX;

      ppb_p->data[buf_index].timestamps[ppb_p->current_index] =
          new_ch101_data.timestamp;

      // i: dev index
      for (uint8_t i = 0; i < HAND_DEV_MAX_NUM_CH101; ++i)
      {
        ppb_p->data[buf_index].data[i][ppb_p->current_index].amp =
            new_ch101_data.data[i].amp;
        ppb_p->data[buf_index].data[i][ppb_p->current_index].range =
            new_ch101_data.data[i].range;
        ppb_p->data[buf_index].data[i][ppb_p->current_index].sample_num =
            new_ch101_data.data[i].sample_num;
      }

      /* increase current_index */
      ++(ppb_p->current_index);

      /* current_index check */
      if (ppb_p->current_index >= HAND_SIZE_PPB_CH101)
      {
        ppb_p->current_index = 0;
        ppb_p->ping_pong_flag = !ppb_p->ping_pong_flag;
      }

      xSemaphoreGive(hand_global_ch101_ping_pong_mutex);
    }
  }
}

void hand_task_ch101_send_data(void* arg)
{
  volatile hand_ppb_ch101_data_t* const ppb_p =
      &hand_global_ch101_ping_pong_buffer;
  uint8_t buffer[HAND_SIZE_NANOPB_BUFFER_CH101] = {0};

  hand_task_arg_ch101_send_data_t* task_arg_p =
      (hand_task_arg_ch101_send_data_t*)arg;

  int client_socket = *task_arg_p;

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
    hand_msg.chip_type = HandChipType_CH101;

    // Set content
    hand_msg.which_content = HandMsg_data_wrapper_tag;

    // Take the mutex before accessing shared resources, may blocked here
    if (xSemaphoreTake(hand_global_ch101_ping_pong_mutex, portMAX_DELAY) ==
        pdTRUE)
    {
      // Determine which buffer to send
      send_buffer_index = ppb_p->ping_pong_flag ? HAND_PPB_SET_BUFFER_INDEX
                                                : HAND_PPB_UNSET_BUFFER_INDEX;
      // Index for storing the next data item
      send_data_index = ppb_p->current_index;
      // For to swap buffers
      ppb_p->current_index = 0;
      ppb_p->ping_pong_flag = !ppb_p->ping_pong_flag;

      // Release the mutex immediately after accessing shared data
      xSemaphoreGive(hand_global_ch101_ping_pong_mutex);
    }

    if (send_data_index != 0)
    {
      ESP_LOGD(TAG, "ch101 send data index is %d", send_data_index);

      /* TODO: could be optimized */
      hand_timestamps_arr_arg_t timestamps_arg = {
          .ts_p = ppb_p->data[send_buffer_index].timestamps,
          .count = send_data_index  // data number
      };

      /* assign every row's first element to iterate every row array in encode
       * function */
      hand_ch101_simple_data_arg_t ch101_data_args[HAND_DEV_MAX_NUM_CH101] = {
          {.d_p = &(ppb_p->data[send_buffer_index].data[0][0]),
           .count = send_data_index},
          {.d_p = &(ppb_p->data[send_buffer_index].data[1][0]),
           .count = send_data_index},
          {.d_p = &(ppb_p->data[send_buffer_index].data[2][0]),
           .count = send_data_index},
          {.d_p = &(ppb_p->data[send_buffer_index].data[3][0]),
           .count = send_data_index}};

      HandDataMsg data_msgs[HAND_DEV_MAX_NUM_CH101];

      /* TODO: should init */

      /* assign same fields */
      for (uint8_t i = 0; i < HAND_DEV_MAX_NUM_CH101; ++i)
      {
        data_msgs[i].data_type = HandDataType_CH101_SIMPLE;
        data_msgs[i].data_count = send_data_index;
        data_msgs[i].has_timestamp = false;
        data_msgs[i].timestamps.funcs.encode = hand_encode_timestamps_array;
        data_msgs[i].timestamps.arg = &timestamps_arg;
        data_msgs[i].data.funcs.encode = hand_encode_ch101_simple_data_array;
        data_msgs[i].data.arg = &ch101_data_args[i];
        data_msgs[i].source = HAND_MSG_SOURCE_CH101_BASE + i;
      }

      // create msg arg
      hand_active_data_msgs_arr_arg_t msg_arg = {
          .msgs_p = data_msgs,
          .max_count = HAND_DEV_MAX_NUM_CH101,
          .active_indicator = &hand_global_ch101_active_dev_num,
          .indicator_type = HandDataType_UINT8};

      hand_msg.content.data_wrapper.data_msgs.funcs.encode =
          hand_encode_active_data_msg_pointers_array;
      hand_msg.content.data_wrapper.data_msgs.arg = &msg_arg;

      /* start to encode */
      int64_t start_time = esp_timer_get_time();

      /* prepare ostream */
      pb_ostream_t stream =
          pb_ostream_from_buffer(buffer, HAND_SIZE_NANOPB_BUFFER_CH101);

      /* encode */
      if (!pb_encode(&stream, HandMsg_fields, &hand_msg))
      {
        ESP_LOGE(TAG, "CH101 msgs encoding failed: %s", PB_GET_ERROR(&stream));
        // XXX: may cause forever loop
        continue;
      }

      int64_t end_time = esp_timer_get_time();
      int64_t encode_duration = end_time - start_time;
      ESP_LOGD(
          TAG,
          "CH101 simple message encoded successfully, size: %zu bytes, time: "
          "%lld us",
          stream.bytes_written, encode_duration);
      hand_overwrite_buf_bytes_count(buffer, stream.bytes_written);

      // Timing start - sending data
      start_time = esp_timer_get_time();

      int ret = send(client_socket, buffer, stream.bytes_written, 0);
      if (ret < 0)
      {
        ESP_LOGE(TAG, "Error occurred during sending CH101 msgs: errno %d",
                 errno);
      }
      else
      {
        ESP_LOGV(TAG, "Message sent (CH101) successfully");
      }

      end_time = esp_timer_get_time();
      int64_t transmit_duration = end_time - start_time;
      ESP_LOGD(TAG, "CH101 Data transmitted successfully, time: %lld us",
               transmit_duration);
    }

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(HAND_MS_CH101_SEND_DATA));
  }
}

void hand_task_alive(void* arg)
{
  /* XXX: move to global states */
  static bool led_on = true;

  /* light up RGB LED blue */
  led_strip_set_pixel(hand_global_devs_handle.rgb_led_handle,
                      HAND_RGB_LED_INDEX, 0, 8, 16);
  led_strip_refresh(hand_global_devs_handle.rgb_led_handle);

  while (1)
  {
    if (xEventGroupGetBits(hand_global_system_event_group) &
        HAND_EG_SYSTEM_LED_CONTROL_BY_ALIVE)
    {
      if (led_on)
      {
        led_strip_clear(hand_global_devs_handle.rgb_led_handle);
        led_strip_refresh(hand_global_devs_handle.rgb_led_handle);
      }
      else
      {
        /* TODO: create hand_led lib */
        led_strip_set_pixel(hand_global_devs_handle.rgb_led_handle,
                            HAND_RGB_LED_INDEX, 0, 8, 16);
        led_strip_refresh(hand_global_devs_handle.rgb_led_handle);
      }
      led_on = !led_on;
    }

    /* access led to blink */
    vTaskDelay(pdMS_TO_TICKS(HAND_MS_ALIVE_BLINK_DELAY / 2));
  }
}