/**
 * \file
 *
 * \brief Board configuration.
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel
 * Support</a>
 */
#ifndef CONF_BOARD_H_INCLUDED
#define CONF_BOARD_H_INCLUDED

/**
 * @brief This file is for hand_espidf board related hardware config
 *
 */

#include "soniclib.h"

/* esp-idf related include */
#include "driver/gpio.h"
#include "driver/i2c.h"

/**
 * Pin config for hand_espidf board
 *
 * IO x: ESP32-S3-MINI
 * CH_x: CH-101 mod 3
 * TCA_x: TCA6408A-CH101 (addr: 0x21), port x
 *
 * CH_INT: active high, no pull up resistor should be put on this line
 *
 *
 * ================== I2C =====================
 * IO 0: I2C-BOARD-SCL,  Direct, Control TCA6408A-CH101
 * IO 1: I2C-BOARD-SDA,  Direct, Control TCA6408A-CH101
 * IO 2: I2C-CH101-SCL,  To TXS0102 B1, Control CH101 Group
 * IO 3: I2C-CH101-SDA,  To TXS0102 B2, Control CH101 Group
 *
 * ================= CH-101 ===================
 * CH1_PROG : TCA_0, Direct
 * CH1_RESET: TCA_1, Direct
 * CH1_INT  : IO 33, Through TXB0104
 * CH2_PROG : TCA_2, Direct
 * CH2_RESET: TCA_3, Direct
 * CH2_INT  : IO 47, Through TXB0104
 * CH3_PROG : TCA_4, Direct
 * CH3_RESET: TCA_5, Direct
 * CH3_INT  : IO 26, Through TXB0104
 * CH4_PROG : TCA_6, Direct
 * CH4_RESET: TCA_7, Direct
 * CH4_INT  : IO 21, Through TXB0104
 *
 * @note User should read the CH101 dev manual to know more about interrupt
 * (read AN-000259)
 * @note The index of CH101 starts from 1
 */

/**
 * \brief Mask to identify TCA6408A pins
 *
 * The CHIRP_IS_TCA6408A_PIN_MASK (1 << 7) is a bitmask used to identify whether
 * a provided pin number corresponds to the TCA6408A I/O expander or the native
 * ESP-IDF GPIOs. By setting the highest bit (bit 7), this mask helps
 * differentiate between the two types of pins. All functions that involve pin
 * operations will first check against this mask to ensure the pin belongs to
 * the TCA6408A system before proceeding.
 *
 * \example
 * if(pin & CHIRP_IS_TCA6408A_PIN_MASK)
 * {
 *   // is TCA6408A pin
 * }
 */
#define CHIRP_IS_TCA6408A_PIN_MASK (1 << 7)
#define CHIRP_GET_TCA6408A_PIN_MASK (~CHIRP_IS_TCA6408A_PIN_MASK)

#define CHIRP_TCA6408A_ADDRESS (0x21)

typedef enum chx01_tca6408a_pin_t
{
  CHX01_1_PROG_TCA6408A_PIN = CHIRP_IS_TCA6408A_PIN_MASK | 0,
  CHX01_1_RESET_TCA6408A_PIN,
  CHX01_2_PROG_TCA6408A_PIN,
  CHX01_2_RESET_TCA6408A_PIN,
  CHX01_3_PROG_TCA6408A_PIN,
  CHX01_3_RESET_TCA6408A_PIN,
  CHX01_4_PROG_TCA6408A_PIN,
  CHX01_4_RESET_TCA6408A_PIN,
} chx01_tca6408a_pin_t;

#define CHIRP_DUMMY_PIN (1 << 6)

/* define reset pin (TCA6408A) */
#define CHIRP_RESET_1 (CHX01_1_RESET_TCA6408A_PIN)
#define CHIRP_RESET_2 (CHX01_2_RESET_TCA6408A_PIN)
#define CHIRP_RESET_3 (CHX01_3_RESET_TCA6408A_PIN)
#define CHIRP_RESET_4 (CHX01_4_RESET_TCA6408A_PIN)

/* define reset pin (TCA6408A) */
#define CHIRP_PROG_1 (CHX01_1_PROG_TCA6408A_PIN)
#define CHIRP_PROG_2 (CHX01_2_PROG_TCA6408A_PIN)
#define CHIRP_PROG_3 (CHX01_3_PROG_TCA6408A_PIN)
#define CHIRP_PROG_4 (CHX01_4_PROG_TCA6408A_PIN)

/* define interrupt pin (esp32 s3) */
#define CHIRP_INT_1 (GPIO_NUM_33)
#define CHIRP_INT_2 (GPIO_NUM_47)
#define CHIRP_INT_3 (GPIO_NUM_26)
#define CHIRP_INT_4 (GPIO_NUM_21)

/* define OK to dummy pin */
#define CHIRP_OK_1 (CHIRP_DUMMY_PIN)
#define CHIRP_OK_2 (CHIRP_DUMMY_PIN)
#define CHIRP_OK_3 (CHIRP_DUMMY_PIN)
#define CHIRP_OK_4 (CHIRP_DUMMY_PIN)

/* define I2C args */
#define CHIRP_I2C_BUS_SPEED (400000)

#define CHIRP_I2C_BUS_0 (I2C_NUM_0)
#define CHIRP_I2C_BUS_1 (I2C_NUM_1)

#define CHIRP_I2C_BUS_0_SCL_PIN (GPIO_NUM_0)
#define CHIRP_I2C_BUS_0_SDA_PIN (GPIO_NUM_1)
#define CHIRP_I2C_BUS_1_SCL_PIN (GPIO_NUM_2)
#define CHIRP_I2C_BUS_1_SDA_PIN (GPIO_NUM_3)

#define CHIRP_I2C_MASTER_TX_BUF_DISABLE (0)
#define CHIRP_I2C_MASTER_RX_BUF_DISABLE (0)

/* define GPIO related args */
#define CHIRP_GPIO_LEVEL_HIGH (1)
#define CHIRP_GPIO_LEVEL_LOW  (0)

/* INTR callback parameter structure wrapper */
typedef struct ch_io_int_callback_parameter_t
{
  ch_group_t* grp_ptr;
  uint8_t io_index;
} ch_io_int_callback_parameter_t;

#endif /* CONF_BOARD_H_INCLUDED */
