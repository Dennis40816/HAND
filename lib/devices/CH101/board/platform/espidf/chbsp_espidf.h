#ifndef LIB_CH101_PLATFORM_ESPIDF_CHBSP_ESPIDF_H_
#define LIB_CH101_PLATFORM_ESPIDF_CHBSP_ESPIDF_H_

/* CHx01 includes */
#include "soniclib.h"
#include "chirp_board_config.h"
#include "app_config.h"

/* esp-idf include */
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

/* TCA6408A include */
#include "tca6408a.h"

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
 */

/* Standard symbols used in board support package - use values from config
 * header */
#define CHBSP_MAX_DEVICES   CHIRP_MAX_NUM_SENSORS
#define CHBSP_NUM_I2C_BUSES CHIRP_NUM_I2C_BUSES

#if defined(CHIRP_RTC_CAL_PULSE_LEN_MS)
/** Length of real-time clock calibration pulse, in milliseconds :
 * length of pulse applied to sensor INT line during clock cal
 */
#define CHBSP_RTC_CAL_PULSE_MS CHIRP_RTC_CAL_PULSE_LEN_MS
#else
/* Default value */
#define CHBSP_RTC_CAL_PULSE_MS (100)
#endif

/* I2C Address assignments for each possible device */
#define CHIRP_I2C_ADDRS \
  {                     \
    45, 43, 44, 42      \
  }
#define CHIRP_I2C_BUSES \
  {                     \
    0, 0, 1, 1          \
  }

/* Structure to track non-blocking I2C transaction data */
typedef struct
{
  uint8_t* buf_ptr;   /* pointer to data buffer */
  uint16_t num_bytes; /* number of bytes to transfer */
} i2c_trans_data_t;

/* INTR callback parameter structure wrapper */
typedef struct
{
  ch_group_t* grp_ptr;
  uint8_t io_index;
} ch_io_int_callback_parameter_t;

/* TC channel used for the ultrasound timer and lsepoch of the system */
/* We use timer group 0 in esp32 s3 */
#define TC_CHANNEL_LSEPOCH (0)
#define TC_CHANNEL_US      (1)

/* Define the HW frequency of the TC used for the ultrasound periodic timer */
#define ULTRASOUND_TIMER_FREQUENCY (500000) /* we set timer freq to 500 kHz */
#define ULTRASOUND_TIMER_DIVIDER   (160)    /* esp32 s3's APB is 80 MHz */

/* Define ESP32 I2C bus speed */
#define I2C_MASTER_FREQ_HZ        400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

/* I2C num macros */
#define CHBSP_I2C_NUM    I2C_NUM_0
#define TCA6408A_I2C_NUM I2C_NUM_1

extern uint32_t chirp_pin_prog[CHBSP_MAX_DEVICES];
extern uint32_t chirp_pin_io[CHBSP_MAX_DEVICES];
extern uint32_t chirp_pin_io_irq[CHBSP_MAX_DEVICES];
extern uint32_t chirp_led_pins[];

extern ch_group_t chirp_group;

extern i2c_trans_data_t
    i2c_nb_transactions[CHBSP_NUM_I2C_BUSES];  // array of structures to track
                                               // non-blocking I2C transactions

extern void sensor_led_on(uint32_t pin);
extern void sensor_led_off(uint32_t pin);
extern void sensor_led_toggle(uint32_t pin);
extern void indicate_alive(void);

extern ch_io_int_callback_t
    io_int_callback_ptr;  // pointer to sensor I/O interrupt callback function

extern void periodic_timer_callback(void);

#endif