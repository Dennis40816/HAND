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
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

/**
 * @brief This file aims to
 *
 */

/*========================= Build Type ===========================*/

/* USER MODIFY: Board Operation Mode Begin */
#define HAND_DEBUG
#ifndef HAND_DEBUG
#define HAND_RELEASE
#endif
/* USER MODIFY:Board Operation Mode End */

#ifdef HAND_DEBUG
#define CHDRV_DEBUG
#endif

/*========================= Build Target ===========================*/

/* USER MODIFY: Test Options Begin */
/**
 * @brief Choose the build target for hand system
 * @warning Choose "ONLY" one of them
 */

// 1. This is for testing the operation of porting a single CH101 to
// the ESP32-S3 platform. For details, please refer to the README.md of the
// folder lib/device/CH101.
#define HAND_BUILD_TARGET_PORTING_CH101

// #define HAND_BUILD_TARGET_ALL

/* USER MODIFY: Test Options End */

/*========================= ESP-IDF config ===========================*/

/* devices pin definitions */
#include "device_config.h"

#endif