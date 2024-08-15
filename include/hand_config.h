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

#include "hand_hw_config.h"
#include "hand_wifi_config.h"

/* CPU ID related config (driver) */
#define HAND_CPU_ID_SPI2 (0)
#define HAND_CPU_ID_SPI3 (0)

/* CPU ID related config (tasks) */

/* Size */

// SPI bus
#define HAND_SIZE_SPI2_TRANSFER      (4096)
#define HAND_SIZE_SPI2_BMI323_QUEUE  (10)
#define HAND_SIZE_SPI2_BOS1901_QUEUE (10)
#define HAND_SIZE_SPI2_KX132_QUEUE   (5)

#define HAND_SIZE_SPI3_TRANSFER    (4096)
#define HAND_SIZE_SPI3_KX132_QUEUE (5)

// Ping pong buffer (PPB)
#define HAND_SIZE_PPB_VL53L1X (35)
#define HAND_SIZE_PPB_CH101   (35)

// NanoPB stream buffer size
#define HAND_SIZE_NANOPB_BUFFER_VL53L1X (2048)

// Queue size (size of the queue used for buffering real-time data before
// storing it in the ping-pong buffer.)
#define HAND_SIZE_QUEUE_VL53L1X (10)
#define HAND_SIZE_QUEUE_CH101   (10)

/* Delay time (ms) */
#define HAND_DELAY_VL53L1X_SEND_DATA (500)
