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
#include "freertos/task.h"

/* Base */

#define HAND_TASK_PRIORITY_HW_BASE (tskIDLE_PRIORITY + 9U)

/* Terminal related */

#define HAND_TASK_PRIORITY_TERMINAL_RECV (tskIDLE_PRIORITY + 1U)

/* VL53L1X related */

#define HAND_TASK_PRIORITY_VL53L1X_COLLECT_DATA (HAND_TASK_PRIORITY_HW_BASE + 1)
#define HAND_TASK_PRIORITY_VL53L1X_FROM_QUEUE_TO_PPB \
  (HAND_TASK_PRIORITY_HW_BASE)
#define HAND_TASK_PRIORITY_VL53L1X_SEND_DATA (HAND_TASK_PRIORITY_HW_BASE + 2)

/* CH101 related */

#define HAND_TASK_PRIORITY_CH101_COLLECT_DATA      (HAND_TASK_PRIORITY_HW_BASE + 1)
#define HAND_TASK_PRIORITY_CH101_FROM_QUEUE_TO_PPB (HAND_TASK_PRIORITY_HW_BASE)
#define HAND_TASK_PRIORITY_CH101_SEND_DATA         (HAND_TASK_PRIORITY_HW_BASE + 2)
