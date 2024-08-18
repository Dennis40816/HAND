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

/**
 * @brief If the CPU ID is specified as 0, an "INSUFFICIENT INTR ALLOCATION"
 * error may occur.
 *
 */
#define HAND_CPU_ID_SPI2 (ESP_INTR_CPU_AFFINITY_1)
#define HAND_CPU_ID_SPI3 (ESP_INTR_CPU_AFFINITY_1)

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
#define HAND_SIZE_NANOPB_BUFFER_CH101   (2048)

// Queue size (size of the queue used for buffering real-time data before
// storing it in the ping-pong buffer.)

#define HAND_SIZE_QUEUE_VL53L1X (10)
#define HAND_SIZE_QUEUE_CH101   (10)

/* Time related (ms) [delay, polling...] */

// VL53L1X related
#define HAND_MS_VL53L1X_QUEUE_MAX_DELAY              (50)
#define HAND_MS_VL53L1X_NEW_DATA_READY_POLL_DURATION (100)
#define HAND_MS_VL53L1X_SEND_DATA                    (500)
// This must be uint16_t [20, 1000]
#define HAND_MS_VL53L1X_DEFAULT_TIMING_BUDGET (50)
// This argument should be equal or larger than (VL53L1X_TIMING_BUDGET_MS +
// 4)
#define HAND_MS_VL53L1X_DEFAULT_MEASURE_PERIOD (100)

// CH101 related
#define HAND_MS_CH101_QUEUE_MAX_DELAY        (50)
#define HAND_MS_CH101_DEFAULT_MEASURE_PERIOD (100)
#define HAND_MS_CH101_SEND_DATA              (500)

// RGB LED related
#define HAND_MS_RGB_LED_BLINK_DELAY (1000)
#define HAND_MS_ALIVE_BLINK_DELAY   HAND_MS_RGB_LED_BLINK_DELAY
