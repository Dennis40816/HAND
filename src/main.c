/***********************************************************************
 * Hello Chirp! - an example application for ultrasonic sensing
 *
 * This project is designed to be your first introduction to using
 * Chirp SonicLib to control ultrasonic sensors in an embedded C
 * application.
 *
 * It configures connected CH101 or CH201 sensors, sets up a measurement
 * timer, and triggers the sensors each time the timer expires.
 * On completion of each measurement, it reads out the sensor data and
 * prints it over the console serial port.
 *
 * The settings used to configure the sensors are defined in
 * the app_config.h header file.
 *
 ***********************************************************************/

/*
 Copyright (c) 2016-2020, Chirp Microsystems.  All rights reserved.
 Copyright (c) 2024, Dennis Liu, dennis48161025@gmail.com. All rights reserved.

 Chirp Microsystems CONFIDENTIAL

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 You can contact the authors of this program by email at support@chirpmicro.com
 or by mail at 2560 Ninth Street, Suite 220, Berkeley, CA 94710.
*/

/* Includes */
// clang-format off
#include <stdio.h>
#include "soniclib.h"			// Chirp SonicLib sensor API definitions
#include "chirp_board_config.h"	// required header with basic device counts etc.
#include "app_config.h"
#include "app_version.h"
#include "chirp_bsp.h"			// board support package function definitions
#include "chirp_smartsonic.h"
#include "ultrasound_display_config_info.h"

/* esp-idf */
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
// clang-format on

/* Bit flags used in main loop to check for completion of sensor I/O.  */
#define DATA_READY_FLAG (1 << 0)
#define IQ_READY_FLAG   (1 << 1)

/* chirp_data_t - Structure to hold measurement data for one sensor
 *   This structure is used to hold the data from one measurement cycle from
 *   a sensor.  The data values include the measured range, the ultrasonic
 *   signal amplitude, the number of valid samples (I/Q data pairs) in the
 *   measurement, and (optionally) the full amplitude data and/or raw I/Q data
 *   from the measurement.
 *
 *  The format of this data structure is specific to this application, so
 *  you may change it as desired.
 *
 *  A "chirp_data[]" array of these structures, one for each possible sensor,
 *  is declared in the main.c file.  The sensor's device number is
 *  used to index the array.
 */
typedef struct
{
  uint32_t range;        // from ch_get_range()
  uint16_t amplitude;    // from ch_get_amplitude()
  uint16_t num_samples;  // from ch_get_num_samples()
#ifdef READ_AMPLITUDE_DATA
  uint16_t amp_data[DATA_MAX_NUM_SAMPLES];
  // from ch_get_amplitude_data()
#endif
#ifdef READ_IQ_DATA
  ch_iq_sample_t iq_data[DATA_MAX_NUM_SAMPLES];
  // from ch_get_iq_data()
#endif
} chirp_data_t;

/* Array of structs to hold measurement data, one for each possible device */
chirp_data_t chirp_data[CHIRP_MAX_NUM_SENSORS];

/* Array of ch_dev_t device descriptors, one for each possible device */
ch_dev_t chirp_devices[CHIRP_MAX_NUM_SENSORS];

/* Configuration structure for group of sensors */
ch_group_t chirp_group;

/* Task flag word
 *   This variable contains the DATA_READY_FLAG and IQ_READY_FLAG bit flags
 *   that are set in I/O processing routines.  The flags are checked in the
 *   main() loop and, if set, will cause an appropriate handler function to
 *   be called to process sensor data.
 */
volatile uint32_t taskflags = 0;

/* Device tracking variables
 *   These are bit-field variables which contain a separate bit assigned to
 *   each (possible) sensor, indexed by the device number.  The active_devices
 *   variable contains the bit pattern describing which ports have active
 *   sensors connected.  The data_ready_devices variable is set bit-by-bit
 *   as sensors interrupt, indicating they have completed a measurement
 *   cycle.  The two variables are compared to determine when all active
 *   devices have interrupted.
 */
static uint32_t active_devices;
static uint32_t data_ready_devices;

/* Number of connected sensors */
static uint8_t num_connected_sensors = 0;

/* Number of sensors that use h/w triggering to start measurement */
static uint8_t num_triggered_devices = 0;

#if (defined(READ_IQ_DATA) && defined(READ_IQ_NONBLOCKING))
/* Count of non-blocking I/Q reads queued */
static uint8_t num_io_queued = 0;
#endif

/* Forward declarations */
static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num);
static void io_complete_callback(ch_group_t *grp_ptr);
static uint8_t handle_data_ready(ch_group_t *grp_ptr);

#ifdef READ_IQ_DATA
static uint8_t display_iq_data(ch_dev_t *dev_ptr);
#ifdef READ_IQ_NONBLOCKING
static uint8_t handle_iq_data_done(ch_group_t *grp_ptr);
#endif
#endif

void app_main(void)
{
  esp_log_level_set("*", ESP_LOG_INFO);
  ch_group_t *grp_ptr = &chirp_group;
  uint8_t chirp_error = 0;
  uint8_t num_ports;
  uint8_t dev_num;

    /* close led */
    gpio_set_direction(GPIO_NUM_48, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_48, 0);

  /* Initialize board hardware functions
   *   This call to the board support package (BSP) performs all necessary
   *   hardware initialization for the application to run on this board.
   *   This includes setting up memory regions, initializing clocks and
   *   peripherals (including I2C and serial port), and any processor-specific
   *   startup sequences.
   *
   *   The chbsp_board_init() function also initializes fields within the
   *   sensor group descriptor, including number of supported sensors and
   *   the RTC clock calibration pulse length.
   */
  vTaskDelay(pdMS_TO_TICKS(1000));
  chbsp_board_init(grp_ptr);

  ESP_LOGI("app_main", "    Hello Chirp! - Chirp SonicLib Example Application");
  ESP_LOGI("app_main", "    Compile time:  %s %s", __DATE__, __TIME__);
  ESP_LOGI("app_main", "    Version: %u.%u.%u", APP_VERSION_MAJOR,
           APP_VERSION_MINOR, APP_VERSION_REV);
  ESP_LOGI("app_main", "    SonicLib version: %u.%u.%u\n", SONICLIB_VER_MAJOR,
           SONICLIB_VER_MINOR, SONICLIB_VER_REV);

  /* Get the number of (possible) sensor devices on the board
   *   Set by the BSP during chbsp_board_init()
   */
  num_ports = ch_get_num_ports(grp_ptr);

  /* Initialize sensor descriptors.
   *   This loop initializes each (possible) sensor's ch_dev_t descriptor,
   *   although we don't yet know if a sensor is actually connected.
   *
   *   The call to ch_init() specifies the sensor descriptor, the sensor group
   *   it will be added to, the device number within the group, and the sensor
   *   firmware initialization routine that will be used.  (The sensor
   *   firmware selection effectively specifies whether it is a CH101 or
   *   CH201 sensor, as well as the exact feature set.)
   */
  ESP_LOGI("app_main", "Initializing sensor(s)... ");

  for (dev_num = 0; dev_num < num_ports; dev_num++)
  {
    ch_dev_t *dev_ptr = &(chirp_devices[dev_num]);  // init struct in array

    /* Init device descriptor
     *   Note that this assumes all sensors will use the same sensor
     *   firmware.  The CHIRP_SENSOR_FW_INIT_FUNC symbol is defined in
     *   app_config.h and is used for all devices.
     *
     *   However, it is possible for different sensors to use different
     *   firmware images, by specifying different firmware init routines
     *   when ch_init() is called for each.
     */
    chirp_error |=
        ch_init(dev_ptr, grp_ptr, dev_num, CHIRP_SENSOR_FW_INIT_FUNC);
  }
}