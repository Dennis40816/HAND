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

/**
 * @brief This document is used to declare all global variables (extern). Each
 * global variable should exist only once and be properly managed with
 * semaphores to control access.
 *
 */

#include "hand_data/hand_data.h"
#include "hand_common.h"
#include "hand_task/hand_task.h"

#include "esp_err.h"

/* HAND related */

extern hand_devices_handle_t hand_global_devs_handle;
extern hand_task_handle_t hand_global_task_handle;

/* CH101 related */

extern volatile uint8_t hand_global_ch101_active_dev_num;
extern volatile uint8_t hand_global_ch101_triggered_dev_num;
extern volatile uint8_t hand_global_ch101_connected_dev_num;

/* Ping pong buffer related */

// VL53L1X related

extern volatile SemaphoreHandle_t hand_global_vl53l1x_ping_pong_mutex;
extern volatile hand_ppb_vl53l1x_data_t hand_global_vl53l1x_ping_pong_buffer;

// CH101 related

extern volatile SemaphoreHandle_t hand_global_ch101_ping_pong_mutex;
extern volatile hand_ppb_ch101_data_t hand_global_ch101_ping_pong_buffer;

/* Queue related */

// VL53L1X related

extern volatile QueueHandle_t hand_global_vl53l1x_data_queue;

// CH101 related

extern volatile QueueHandle_t hand_global_ch101_data_queue;

/* Event group (EG) related */

// VL53L1X related

#define HAND_EG_VL53L1X_1_DATA_READY_BIT (1 << 0)
#define HAND_EG_VL53L1X_2_DATA_READY_BIT (1 << 1)
#define HAND_EG_VL53L1X_1_FAILURE_BIT    (1 << 2)
#define HAND_EG_VL53L1X_2_FAILURE_BIT    (1 << 3)

extern volatile EventGroupHandle_t hand_global_vl53l1x_event_group;

// CH101 related

#define HAND_EG_CH101_1_DATA_READY_BIT              (1 << 0)
#define HAND_EG_CH101_2_DATA_READY_BIT              (1 << 1)
#define HAND_EG_CH101_3_DATA_READY_BIT              (1 << 2)
#define HAND_EG_CH101_4_DATA_READY_BIT              (1 << 3)
#define HAND_EG_CH101_ALL_ACTIVE_DEV_DATA_READY_BIT (1 << 4)
#define HAND_EG_CH101_IQ_DATA_READY                 (1 << 5)

extern volatile EventGroupHandle_t hand_global_ch101_event_group;

// HAND system related

#define HAND_EG_SYSTEM_BATTERY_PLUG_BIT     (1 << 0)
#define HAND_EG_SYSTEM_USB_PLUG_BIT         (1 << 1)
#define HAND_EG_SYSTEM_LED_CONTROL_BY_ALIVE (1 << 2)

extern volatile EventGroupHandle_t hand_global_system_event_group;

/* public API */

/**
 * @brief Call in hand_init()
 *
 * @return esp_err_t
 */
esp_err_t hand_global_var_init();

/* Callbacks related */

// VL53L1X related
void IRAM_ATTR hand_cb_vl53l1x_sensed(void* arg);

// CH101 related
void IRAM_ATTR hand_cb_ch101_sensed(ch_group_t* grp_ptr, uint8_t dev_num);

/**
 * @brief Non-blocking I/O complete callback routine
 *
 * This function is called by SonicLib's I2C DMA handling function when all
 * outstanding non-blocking I/Q readouts have completed.
 *
 * This callback function is registered by the call to
 * ch_io_complete_callback_set() in main().
 *
 * @note This callback is only used if READ_IQ_NONBLOCKING is defined to
 *  select non-blocking I/Q readout in this application.
 *
 * @param grp_ptr
 */
void IRAM_ATTR hand_cb_ch101_io_completed(ch_group_t* grp_ptr);

void IRAM_ATTR hand_cb_ch101_periodic_timer(void);