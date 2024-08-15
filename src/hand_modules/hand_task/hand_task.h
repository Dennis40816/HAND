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

#include "freertos/FreeRTOS.h"

/* Public macro */

/* TODO: stack size (SS), needs optimization */

// Terminal related

#define HAND_TASK_SS_TERMINAL_RECV (4096)

// VL53L1X related

#define HAND_TASK_SS_VL53L1X_COLLECT_DATA      (4096)
#define HAND_TASK_SS_VL53L1X_FROM_QUEUE_TO_PPB (4096)
#define HAND_TASK_SS_VL53L1X_SEND_DATA         (8192)

/* Public struct */

/**
 * @brief Take only a socket as arg
 *
 */
typedef int hand_task_arg_vl53l1x_send_data_t;

typedef struct hand_task_handle_t
{
  TaskHandle_t vl53l1x_collect_data_handle;
  TaskHandle_t vl53l1x_from_queue_to_ppb_handle;
  TaskHandle_t vl53l1x_send_data_handle;
  /* TODO: not implement yet */
} hand_task_handle_t;

/* Public API */

/**
 * @brief Interacts with the device struct and hardware bus (I2C 0) to collect
 * data and place it into the queue.
 *
 * This task is designed to perform actual interactions with the device
 * structure and the hardware bus (I2C 0). It collects data from the VL53L1X
 * device and places the retrieved data into the queue for further processing.
 * The `arg` parameter is currently unused.
 *
 * @param arg Currently unused.
 */
void hand_task_vl53l1x_collect_data(void* __attribute__((unused)) arg);

/**
 * @brief Continuously retrieves VL53L1X data units from the queue and stores
 * them into the ping-pong buffer.
 *
 * This task is responsible for extracting VL53L1X data units from the queue and
 * storing them in the ping-pong buffer. When the ping-pong buffer becomes full,
 * it switches the active storage buffer to ensure continuous data flow. The
 * `arg` parameter is currently unused.
 *
 * @param arg Currently unused.
 */
void hand_task_vl53l1x_from_queue_to_ppb(void* __attribute__((unused)) arg);

// TODO Consider adding a flag within `arg` in the future to determine whether
// this task should be restarted.
/**
 * @brief Encodes data from the ping-pong buffer into a HAND_MSG format and
 * sends it to the target server.
 *
 * This task retrieves data from the ping-pong buffer, encodes it into the
 * HAND_MSG format, and transmits it to the specified server. The `arg`
 * parameter is expected to be of type `hand_task_arg_vl53l1x_send_data_t*`,
 * which contains the necessary information for the task execution.
 *
 * @param arg A pointer to a structure of type
 * `hand_task_arg_vl53l1x_send_data_t` that provides the configuration and
 * control information for this task.
 */
void hand_task_vl53l1x_send_data(void* arg);

void hand_task_ch101_from_queue_to_ppb(void* __attribute__((unused)) arg);

void hand_task_ch101_send_data(void* arg);
