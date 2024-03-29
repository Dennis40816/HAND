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

/* esp-idf gpio definition include */
#include "driver/gpio.h"

// #ifndef CONF_BOARD_UART_CONSOLE
// #define CONF_BOARD_UART_CONSOLE
// #endif

#ifndef CONF_BOARD_USART0
#define CONF_BOARD_USART0
#endif

#ifndef BOARD_FLEXCOM_SPI
/** FLEXCOM base address for SPI mode*/
#define BOARD_FLEXCOM_SPI FLEXCOM5
#endif

// #ifndef BOARD_FLEXCOM_USART
/** FLEXCOM base address for USART mode*/
// #define BOARD_FLEXCOM_USART  FLEXCOM6
// #endif

/* I2C defines - I2C_FLAG */

/** Configure TWI4 pins */
#define CONF_BOARD_TWI4
#define CONF_BOARD_TWI1
#define CONF_BOARD_TWI3

/** Flexcom application to use */
#define BOARD_FLEXCOM_TWI4 FLEXCOM4
#define BOARD_FLEXCOM_TWI1 FLEXCOM1
#define BOARD_FLEXCOM_TWI3 FLEXCOM3

/** TWI ID for simulated EEPROM application to use */
#define BOARD_ID_TWI ID_TWI4

/** TWI Base for simulated TWI EEPROM application to use */
#define BOARD_BASE_TWI4 TWI4
#define BOARD_BASE_TWI1 TWI1
#define BOARD_BASE_TWI3 TWI3

/** SPI MACRO definition */
#define CONF_BOARD_SPI  // sheena 7-2-2018 for I2C, this must be commented

/** SPI slave select MACRO definition */
#define CONF_BOARD_SPI_NPCS0

/** Spi Hw ID . */
#define SPI_ID ID_SPI5

/** SPI base address for SPI master mode*/
#define SPI_MASTER_BASE SPI5
/** SPI base address for SPI slave mode, (on different board) */
#define SPI_SLAVE_BASE SPI5
/** FLEXCOM base address for SPI mode*/
#define BOARD_FLEXCOM_SPI FLEXCOM5

/*External Interrupt setup for PA30 Motion Sensor INT*/
#define PIN_EXT_MotionINT_MASK PIO_PA30

/*External Interrupt setup for CHIRP_INT_0/1/2/3 INT*/
#define PIN_EXT_ChirpINT0_MASK PIO_PA21
#define PIN_EXT_ChirpINT1_MASK PIO_PA22
#define PIN_EXT_ChirpINT2_MASK PIO_PA23
#define PIN_EXT_ChirpINT3_MASK PIO_PA29

#define PIN_EXT_INTERRUPT_PIO  PIOA
#define PIN_EXT_INTERRUPT_ID   ID_PIOA
#define PIN_EXT_INTERRUPT_TYPE PIO_INPUT
#define PIN_EXT_INTERRUPT_ATTR (PIO_DEFAULT | PIO_IT_RISE_EDGE)
#define PIN_EXT_INTERRUPT_IRQn PIOA_IRQn

/*for chirp chip control*/
/**
 * Pin config for HAND_BUILD_TARGET_PORTING_CH101
 *
 * IO x: ESP32-S3-MINI
 * CH-x: CH-101 mod 3
 *
 * IO 0: TCA6408A RESET, Direct
 * IO 4: TCA6408A SDA,   Direct
 * IO 5: TCA6408A SCL,   Direct
 * IO 6: TCA6408A INT,   Direct (only ch101 int will trigger it, a.k.a CH-INT
 * input, need to config TCA6408A P5 to input mode)
 *
 * CH-INT output: TCA6408A P5, Through I2C_NUM_1
 * CH-RESET output: TCA6408A P6, Through I2C_NUM_1
 * CH-PROG output: TCA6408A P7, Through I2C_NUM_1
 *
 * CH-SDA - TXS0102 B2 - IO 12 (I2C_NUM_0), Direct
 * CH-SCL - TXS0102 B1 - IO 13 (I2C_NUM_0), Direct
 *
 * Note that for INT line, the symbol is always CH101_INT_TCA6408A_PIN (P5 and
 * IO 6)
 */

/* Define TCA6408A - CH-101 related enums */
/* WARNING: 1 << 7 should always be 1 (for GPIO_NUM_X & (1<< 7) always 0) */

/* usage: CH101_INT_TCA6408A_PIN & CH101_TCA_GET_PIN_MASK -> get 5 (means p5) */
#define CH101_TCA_PIN_MASK     (1 << 7)
#define CH101_TCA_GET_PIN_MASK ((uint8_t)~CH101_TCA_PIN_MASK)

typedef enum {
  CH101_INT_TCA6408A_PIN = CH101_TCA_PIN_MASK | 5,  // TCA6408A P5 -> CH101_INT
  CH101_RESET_TCA6408A_PIN =
      CH101_TCA_PIN_MASK | 6,  // TCA6408A P6 -> CH101_RESET
  CH101_PROG_TCA6408A_PIN =
      CH101_TCA_PIN_MASK | 7,  // TCA6408A P7 -> CH101_PROG
} ch101_tca6408a_pin_t;

#define PIN_EXT_MotionINT IOPORT_CREATE_PIN(PIOA, 30)  // PIO_PA30
#define CHIRP_RST         CH101_RESET_TCA6408A_PIN
#define CHIRP_PROG_0      CH101_PROG_TCA6408A_PIN
#define CHIRP_INT_0       CH101_INT_TCA6408A_PIN
#define CHIRP_OK_0        GPIO_NUM_48

/* interrupt of esp32 */
#define CHIRP_INT_IO6     GPIO_NUM_6

/* define a dummy pin to pass compile procedure */
#define DUMMY_PIN         GPIO_NUM_10

/* not use pin definition */
#define CHIRP_PROG_1 DUMMY_PIN
#define CHIRP_PROG_2 DUMMY_PIN
#define CHIRP_PROG_3 DUMMY_PIN

#define CHIRP_INT_1 DUMMY_PIN
#define CHIRP_INT_2 DUMMY_PIN
#define CHIRP_INT_3 DUMMY_PIN

#define CHIRP_OK_1 DUMMY_PIN
#define CHIRP_OK_2 DUMMY_PIN
#define CHIRP_OK_3 DUMMY_PIN

#endif /* CONF_BOARD_H_INCLUDED */
