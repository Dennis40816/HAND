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

#include "driver/gpio.h"

/* TODO: should hide the predefine and only export user should use macros */
/**
 * @brief HAND hardware configuration, please ref to sch pdf (hand_main_v2.pdf)
 *
 * @note  - KX132 -> KX132-1211 (accelerometer)
 * @note  - TCA -> TCA6408A (IO expander)
 * @note  - BQ25302 -> Battery charger
 * @note  - BQ27427 -> Battery fuel gauge
 * @note  - TXB0104 -> GPIO level shifter
 * @note  - TXS0102 -> I2C bus level shifter
 *
 * @details devices
 * spi devices on SPI-2
 * - BOS1901_1
 * - BOS1901_2
 * - BOS1901_3
 * - BOS1901_4
 * - BMI323
 * - KX132_1
 *
 * spi devices on SPI-3
 * - KX132_2
 * - KX132_3
 * - KX132_4
 *
 * i2c devices on I2C-0 (I2C OTHER)
 * - TCA6408A OTHER
 * - TCA6408A CH101
 * - BQ27427
 * - VL53L1X-1
 * - VL53L1X-2
 *
 * i2c devices on I2C-1 (I2C CH101)
 * - CH101_1
 * - CH101_2
 * - CH101_3
 * - CH101_4
 */
#define HAND_FIRMWARE_VERSION "1.0.1"
/* gpio level related */

#define HAND_GPIO_LEVEL_LOW        (0)
#define HAND_GPIO_LEVEL_HIGH       (1)
#define HAND_GPIO_DUMMY_PIN        (-1)
#define HAND_BQ25302_EN_ENABLE     HAND_GPIO_LEVEL_LOW
#define HAND_BQ25302_EN_DISABLE    HAND_GPIO_LEVEL_HIGH
#define HAND_VBUS_DETECT_PLUG      HAND_GPIO_LEVEL_HIGH
#define HAND_VBUS_DETECT_UNPLUG    HAND_GPIO_LEVEL_LOW
#define HAND_BATTERY_DETECT_PLUG   HAND_GPIO_LEVEL_HIGH
#define HAND_BATTERY_DETECT_UNPLUG HAND_GPIO_LEVEL_LOW

/* I2C related */
#define HAND_PIN_I2C_OTHER_SCL (GPIO_NUM_0)
#define HAND_PIN_I2C_OTHER_SDA (GPIO_NUM_1)
#define HAND_PIN_I2C_CH101_SCL (GPIO_NUM_2)
#define HAND_PIN_I2C_CH101_SDA (GPIO_NUM_3)

/* SPI related */
// SPI-2 (FSPI)
// Issue: This pin can be accessed via IO_MUX or GPIO_MATRIX. However, due to
// the use of an incorrect IO number (hardware problem and can't be fixed from
// software), it can only be accessed via GPIO_MATRIX. See datasheet P.16
#define HAND_PIN_SPI_2_MISO (GPIO_NUM_11)  // should be MOSI if using IO_MUX
#define HAND_PIN_SPI_2_SCLK (GPIO_NUM_12)
#define HAND_PIN_SPI_2_MOSI (GPIO_NUM_13)  // should be MISO if using IO_MUX
#define HAND_PIN_SPI_2_SDI  HAND_PIN_SPI_2_MOSI
#define HAND_PIN_SPI_2_SDO  HAND_PIN_SPI_2_MISO

// SPI-3 (VSPI)
// Can be accessed only via GPIO_MATRIX !
#define HAND_PIN_SPI_3_MISO (GPIO_NUM_40)
#define HAND_PIN_SPI_3_SCLK (GPIO_NUM_41)
#define HAND_PIN_SPI_3_MOSI (GPIO_NUM_42)
#define HAND_PIN_SPI_3_SDI  HAND_PIN_SPI_3_MOSI
#define HAND_PIN_SPI_3_SDO  HAND_PIN_SPI_3_MISO

/* SPI CS (SPI-2) */
#define HAND_PIN_CS_BMI323    (GPIO_NUM_5)
#define HAND_PIN_CS_KX132_1   (GPIO_NUM_6)
#define HAND_PIN_CS_BOS1901_4 (GPIO_NUM_7)
#define HAND_PIN_CS_BOS1901_3 (GPIO_NUM_8)
#define HAND_PIN_CS_BOS1901_2 (GPIO_NUM_9)
#define HAND_PIN_CS_BOS1901_1 (GPIO_NUM_10)

/* SPI CS (SPI-3) */
#define HAND_PIN_CS_KX132_4 (GPIO_NUM_35)
#define HAND_PIN_CS_KX132_3 (GPIO_NUM_37)
#define HAND_PIN_CS_KX132_2 (GPIO_NUM_39)

/* TCA6408 PIN related flags */
#define HAND_PIN_TCA_CH101_FLAG (1 << 7)
#define HAND_PIN_TCA_OTHER_FLAG (1 << 6)  // a.k.a TCA_BOARD

#define HAND_PIN_TCA_CH101_P0 (HAND_PIN_TCA_CH101_FLAG | 0)
#define HAND_PIN_TCA_CH101_P1 (HAND_PIN_TCA_CH101_FLAG | 1)
#define HAND_PIN_TCA_CH101_P2 (HAND_PIN_TCA_CH101_FLAG | 2)
#define HAND_PIN_TCA_CH101_P3 (HAND_PIN_TCA_CH101_FLAG | 3)
#define HAND_PIN_TCA_CH101_P4 (HAND_PIN_TCA_CH101_FLAG | 4)
#define HAND_PIN_TCA_CH101_P5 (HAND_PIN_TCA_CH101_FLAG | 5)
#define HAND_PIN_TCA_CH101_P6 (HAND_PIN_TCA_CH101_FLAG | 6)
#define HAND_PIN_TCA_CH101_P7 (HAND_PIN_TCA_CH101_FLAG | 7)

#define HAND_PIN_TCA_OTHER_P0 (HAND_PIN_TCA_OTHER_FLAG | 0)
#define HAND_PIN_TCA_OTHER_P1 (HAND_PIN_TCA_OTHER_FLAG | 1)
#define HAND_PIN_TCA_OTHER_P2 (HAND_PIN_TCA_OTHER_FLAG | 2)
#define HAND_PIN_TCA_OTHER_P3 (HAND_PIN_TCA_OTHER_FLAG | 3)
#define HAND_PIN_TCA_OTHER_P4 (HAND_PIN_TCA_OTHER_FLAG | 4)
#define HAND_PIN_TCA_OTHER_P5 (HAND_PIN_TCA_OTHER_FLAG | 5)
#define HAND_PIN_TCA_OTHER_P6 (HAND_PIN_TCA_OTHER_FLAG | 6)
#define HAND_PIN_TCA_OTHER_P7 (HAND_PIN_TCA_OTHER_FLAG | 7)

/* TCA CH101 pin config */
#define HAND_PIN_TCA_CH101_1_PROG  HAND_PIN_TCA_CH101_P0  // I/O
#define HAND_PIN_TCA_CH101_1_RESET HAND_PIN_TCA_CH101_P1  // O only
#define HAND_PIN_TCA_CH101_2_PROG  HAND_PIN_TCA_CH101_P2  // I/O
#define HAND_PIN_TCA_CH101_2_RESET HAND_PIN_TCA_CH101_P3  // O only
#define HAND_PIN_TCA_CH101_3_PROG  HAND_PIN_TCA_CH101_P4  // I/O
#define HAND_PIN_TCA_CH101_3_RESET HAND_PIN_TCA_CH101_P5  // O only
#define HAND_PIN_TCA_CH101_4_PROG  HAND_PIN_TCA_CH101_P6  // I/O
#define HAND_PIN_TCA_CH101_4_RESET HAND_PIN_TCA_CH101_P7  // O only

/* TCA CH101 pin config */
#define HAND_PIN_TCA_BQ27427_GPOUT   HAND_PIN_TCA_OTHER_P0  // I/O
#define HAND_PIN_TCA_BQ25302_EN      HAND_PIN_TCA_OTHER_P1  // O only
#define HAND_PIN_TCA_BQ25302_STAT    HAND_PIN_TCA_OTHER_P2  // I only
#define HAND_PIN_TCA_VL53L1X_1_XSHUT HAND_PIN_TCA_OTHER_P3  // O only
#define HAND_PIN_TCA_BMI323_INT_2    HAND_PIN_TCA_OTHER_P4  // I INT only
#define HAND_PIN_TCA_VL53L1X_2_XSHUT HAND_PIN_TCA_OTHER_P5  // O only
#define HAND_PIN_TCA_VBUS_DETECT     HAND_PIN_TCA_OTHER_P6  // I only
#define HAND_PIN_TCA_BATTERY_DETECT  HAND_PIN_TCA_OTHER_P7  // I only

/* INT related */
#define HAND_PIN_INT_BMI323_INT_1    (GPIO_NUM_4)
#define HAND_PIN_INT_BMI323_INT_2    HAND_PIN_TCA_BMI323_INT_2
#define HAND_PIN_INT_TCA_OTHER_INT   (GPIO_NUM_14)
#define HAND_PIN_INT_VL53L1X_1_GPIO1 (GPIO_NUM_17)
#define HAND_PIN_INT_VL53L1X_2_GPIO1 (GPIO_NUM_18)
#define HAND_PIN_INT_TXB0104_B4      (GPIO_NUM_21)
#define HAND_PIN_INT_TXB0104_B3      (GPIO_NUM_26)
#define HAND_PIN_INT_TXB0104_B1      (GPIO_NUM_33)
#define HAND_PIN_INT_KX132_1_INT1    (GPIO_NUM_34)
#define HAND_PIN_INT_KX132_3_INT1    (GPIO_NUM_36)
#define HAND_PIN_INT_KX132_2_INT1    (GPIO_NUM_38)
#define HAND_PIN_INT_TXB0104_B2      (GPIO_NUM_47)
#define HAND_PIN_INT_KX132_4_INT1    (GPIO_NUM_48)
#define HAND_PIN_INT_CH101_1_INT     HAND_PIN_INT_TXB0104_B1
#define HAND_PIN_INT_CH101_2_INT     HAND_PIN_INT_TXB0104_B2
#define HAND_PIN_INT_CH101_3_INT     HAND_PIN_INT_TXB0104_B3
#define HAND_PIN_INT_CH101_4_INT     HAND_PIN_INT_TXB0104_B4

/* Debug related (include LED) */
#define HAND_PIN_DEBUG_P1      (GPIO_NUM_43)
#define HAND_PIN_DEBUG_P2      (GPIO_NUM_44)
#define HAND_PIN_DEBUG_P3      (GPIO_NUM_45)
#define HAND_PIN_DEBUG_P4      (GPIO_NUM_46)
#define HAND_PIN_DEBUG_TXD0    HAND_PIN_DEBUG_P1
#define HAND_PIN_DEBUG_RXD0    HAND_PIN_DEBUG_P2
#define HAND_PIN_DEBUG_RGB_LED HAND_PIN_DEBUG_P3

/* Bus related config (address, speed, mode...). Needs test */
// Speed
#define HAND_BUS_I2C_0_SPEED       (400000)    // 400k
#define HAND_BUS_I2C_1_SPEED       (400000)    // 400k
#define HAND_BUS_SPI_BOS1901_SPEED (32000000)  // 32M (Ideal Max 35M)
#define HAND_BUS_SPI_KX132_SPEED   (10000000)  // 10M  (Ideal 10M)
#define HAND_BUS_SPI_BMI323_SPEED  (10000000)  // 10M  (Ideal 10M)

// SPI mode (CPOL, CPHA)
#define HAND_BUS_SPI_MODE_0       (0)  // (0, 0)
#define HAND_BUS_SPI_MODE_1       (1)  // (0, 1)
#define HAND_BUS_SPI_MODE_2       (2)  // (1, 0)
#define HAND_BUS_SPI_MODE_3       (3)  // (1, 1)
#define HAND_BUS_SPI_BOS1901_MODE HAND_BUS_SPI_MODE_0
#define HAND_BUS_SPI_KX132_MODE   HAND_BUS_SPI_MODE_0
#define HAND_BUS_SPI_BMI323_MODE  HAND_BUS_SPI_MODE_0

// TODO: RW flags

// I2C address (7-bit address, need shift 1 bit, e.g., addr << 1 |
// RW_FLAG)

// DEFAULT
#define HAND_BUS_I2C_ADDR_CH101_APP_DEFAULT (0x29)
#define HAND_BUS_I2C_ADDR_VL53L1X_DEFAULT   (0x52)  // XXX: It's 8 bit address

// Fixed
#define HAND_BUS_I2C_ADDR_TCA_OTHER  (0x20)
#define HAND_BUS_I2C_ADDR_TCA_CH101  (0x21)
#define HAND_BUS_I2C_ADDR_BQ27427    (0x55)
#define HAND_BUS_I2C_ADDR_CH101_PROG (0x45)

// @USER: Modifiable
#define HAND_BUS_I2C_ADDR_CH101_1 (0x2A)  // 42
#define HAND_BUS_I2C_ADDR_CH101_2 (0x2B)  // 43
#define HAND_BUS_I2C_ADDR_CH101_3 (0x2C)  // 44
#define HAND_BUS_I2C_ADDR_CH101_4 (0x2D)  // 45

// XXX: Currently, VL53L1X use 8 bit address. Needs to refactor later
#define HAND_BUS_I2C_ADDR_VL53L1X_1 (0x60)
#define HAND_BUS_I2C_ADDR_VL53L1X_2 (0x62)
