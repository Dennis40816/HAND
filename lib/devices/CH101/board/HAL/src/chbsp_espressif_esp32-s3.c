/*
 Copyright (c) 2016-2019, Chirp Microsystems.  All rights reserved.
 Copyright (c) 2024, Dennis Liu, dennis48161025@gmail.com. All right reserved.

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

// clang-format off

/* header with board-specific defines, e.g., "chirp_smartsonic.h" for smartsonic
 * board */
#include "conf_board.h"
#include "chbsp_espressif_esp32-s3.h"

/* esp-idf header */
#include "esp_timer.h"
#include "rom/ets_sys.h"

/* FREERTOS */
#include "freertos/FreeRTOS.h"

/* chx01 lib */
#include "soniclib.h"
#include "chirp_bsp.h"

// clang-format on

/* Define GPIO level macros */
#define GPIO_LEVEL_LOW  (0)
#define GPIO_LEVEL_HIGH (1)

static uint8_t chirp_i2c_addrs[] = CHIRP_I2C_ADDRS;
static uint8_t chirp_i2c_buses[] = CHIRP_I2C_BUSES;

/*
 * Here you set the pin masks for each of the prog pins
 * TODO: update related gpio
 */
uint32_t chirp_pin_prog[] = CHIRP_PIN_PROG;
uint32_t chirp_pin_io[] = CHIRP_PIN_IO;
// uint32_t chirp_pin_io_irq[] = CHIRP_PIN_IO_IRQ;
uint32_t chirp_led_pins[] = CHIRP_PIN_LED;

/* Chirp sensor group pointer */
ch_group_t* sensor_group_ptr;

/* Parameter storage for sensor 0 */
ch_io_int_callback_parameter_t ch_io_it_cb_para0 = {.grp_ptr = NULL,
                                                    .io_index = 0};

/* TCA6408A settings */
const tca6408a_t tca6408a_config = {.i2c_port = TCA6408A_I2C_NUM};

/* Callback function pointers */
static esp_timer_handle_t periodic_timer_handle_ptr = NULL;
static ch_timer_callback_t periodic_timer_callback_ptr = NULL;

static uint32_t periodic_timer_interval_us;

/* esp-idf doesn't need this */
// static uint16_t ultrasound_timer_period_in_tick = 0xFFFF;*
// static uint16_t ultrasound_prev_period_end_in_tick;

/* Counter used to decimate call to ultrasound timer callback from TC0 ISR in
   case decimation factor is != 1 */
static uint8_t decimation_counter = 0;

/* Used in case the timer resolution and range (16 bits HW counter) overflows */
static uint8_t decimation_factor;

/* Forward declaration */
// static void program_next_period(void);
// static uint32_t get_period_in_tick(uint32_t interval_us);
static void ext_int_init(void);
static void find_sensors(void);

/* =============== Functions ============== */

/**
 * @brief Set the direction of a GPIO pin through direct GPIO manipulation.
 *
 * This function configures the direction of a specified GPIO pin. It handles
 * a special case for an interrupt line by re-enabling the interrupt if the
 * pin is configured as an input. It avoids resetting the pin to maintain
 * interrupt flags.
 *
 * @param gpio_num The GPIO number to configure.
 * @param gpio_mode The mode to set for the GPIO (input, output, etc.).
 *
 * @example ioport_set_pin_dir_through_gpio(GPIO_NUM_1, GPIO_MODE_INPUT)
 * @note For interrupt line (IO 6), re-enable the interrupt if configuring as
 * input.
 */
static void ioport_set_pin_dir_through_gpio(gpio_num_t gpio_num,
                                            gpio_mode_t gpio_mode)
{
  // gpio_reset_pin(gpio_num); // will clean intr flag, not good for ISR

  /* special case for INT line */
  if (gpio_mode == GPIO_MODE_INPUT && gpio_num == CHIRP_INT_IO6)
  {
    // enable interrupt again
    gpio_intr_enable(CHIRP_INT_IO6);
  }
  gpio_set_direction(gpio_num, gpio_mode);
  ESP_LOGD("ioport_set_pin_dir_through_gpio", "Set gpio num: %d to mode: %d.",
           (int)gpio_num, (int)gpio_mode);
}

/**
 * @brief Set the direction of a pin connected through a TCA6408A I/O expander.
 *
 * This function configures the direction of a pin that is connected through
 * a TCA6408A I/O expander chip. It supports only input and output modes.
 * The function calculates the correct register value for the TCA6408A to
 * configure the pin direction.
 *
 * @param pin See ch101_tca6408a_pin_t
 * @param mode The mode to set for the pin (input or output).
 *
 * @example ioport_set_pin_dir_through_tca6408a(CHIRP_RST, GPIO_MODE_INPUT)
 * @note This function supports only GPIO_MODE_INPUT and GPIO_MODE_OUTPUT.
 */
static void ioport_set_pin_dir_through_tca6408a(ch101_tca6408a_pin_t pin,
                                                gpio_mode_t mode)
{
  /* check mode is whether GPIO_MODE_INPUT or GPIO_MODE_OUTPUT */
  if (mode != GPIO_MODE_INPUT && mode != GPIO_MODE_OUTPUT)
  {
    ESP_LOGE("ioport_set_pin_dir_through_tca6408a",
             "mode: %d, not input mode nor output mode. Abort!", mode);
    return;
  }

  /* get real pin of TCA6408A and shift it to fit the CONFIG register (0x04)
   * format */
  uint8_t pin_reg = 1 << (pin & CH101_TCA_GET_PIN_MASK);

  if (mode == GPIO_MODE_OUTPUT)
  {
    tca6408a_set_output(pin_reg, &tca6408a_config);
  }
  else
  {
    tca6408a_set_input(pin_reg, &tca6408a_config);
  }
  ESP_LOGD("ioport_set_pin_dir_through_tca6408a", "Set P%d to %s.",
           pin & CH101_TCA_GET_PIN_MASK,
           (mode == GPIO_MODE_OUTPUT) ? "output" : "input");
}

/**
 * @brief Configure the direction of a general-purpose I/O pin.
 *
 * This function is the external API used for configuring the direction of
 * both direct GPIO pins and pins accessed through a TCA6408A I/O expander.
 * It includes special handling for interrupt lines, disabling the interrupt
 * when the pin is configured as output, and setting the corresponding
 * direct GPIO for input configurations.
 *
 * @param pin The pin to configure. Can be a direct GPIO or a TCA6408A pin.
 * @param mode The mode to set for the pin (input, output, etc.).
 *
 * @example ioport_set_pin_dir(CHIRP_RST, GPIO_MODE_INPUT)
 * @example ioport_set_pin_dir(GPIO_NUM_3, GPIO_MODE_OUTPUT)
 * @note Special handling for interrupt lines and TCA6408A pins.
 */
static void ioport_set_pin_dir(uint8_t pin, gpio_mode_t mode)
{
  /* special case for INT line */
  if (pin == CHIRP_INT_0)
  {
    ioport_set_pin_dir_through_tca6408a((ch101_tca6408a_pin_t)pin, mode);

    /* CHIRP_INT_IO6 should be configured too */
    if (mode == GPIO_MODE_OUTPUT)
    {
      gpio_intr_disable(CHIRP_INT_IO6);
    }
    else if (mode == GPIO_MODE_INPUT)
    {
      ioport_set_pin_dir_through_gpio(CHIRP_INT_IO6, mode);
    }
    else
    {
      /* No other mode should be called */
      ESP_LOGE("ioport_set_pin_dir",
               "Mode error: %d when setting pin INT. Abort!", (int)mode);
    }
    return;
  }

  // if pin is CH101_TCA_PIN format (7th bit is 1)
  if (pin & CH101_TCA_PIN_MASK)
  {
    ioport_set_pin_dir_through_tca6408a((ch101_tca6408a_pin_t)pin, mode);
  }
  else
  {
    ioport_set_pin_dir_through_gpio((gpio_num_t)pin, mode);
  }
}

/**
 * @brief Set the level of a GPIO pin through direct GPIO manipulation.
 *
 * This function sets the specified GPIO pin to a desired level (high or low).
 * It logs the action for debugging purposes.
 *
 * @param gpio_num The GPIO number to set the level of.
 * @param val The level to set the GPIO to, where 0 is low and 1 is high.
 *
 * @example ioport_set_pin_level_through_gpio(GPIO_NUM_5, GPIO_LEVEL_HIGH); //
 * Set GPIO5 to high
 */
static void ioport_set_pin_level_through_gpio(gpio_num_t gpio_num, uint32_t val)
{
  gpio_set_level(gpio_num, val);
  ESP_LOGD("ioport_set_pin_level_through_gpio", "Set gpio num: %d to %s.",
           (int)gpio_num, (val == GPIO_LEVEL_HIGH) ? "high" : "low");
}

/**
 * @brief Set the level of a pin connected through a TCA6408A I/O expander.
 *
 * This function sets the level (high or low) of a pin that is connected through
 * a TCA6408A I/O expander chip. It supports setting individual pins to high or
 * low. The function determines the correct pin register configuration for the
 * TCA6408A to set the pin level.
 *
 * @param pin The pin on the TCA6408A to set the level of (0-7).
 * @param val The level to set the pin to, where 0 is low and 1 is high.
 *
 * @example ioport_set_pin_level_through_tca6408a(CH101_INT_TCA6408A_PIN,
 * GPIO_LEVEL_LOW); // Set TCA6408A P5(connect to CH-101 INT) to low
 */
static void ioport_set_pin_level_through_tca6408a(ch101_tca6408a_pin_t pin,
                                                  uint32_t val)
{
  /* get real pin of TCA6408A and shift it to fit the CONFIG register (0x04)
   * format */
  uint8_t pin_reg = 1 << (pin & CH101_TCA_GET_PIN_MASK);

  if (val == GPIO_LEVEL_HIGH)
  {
    tca6408a_set_high(pin_reg, pin_reg, &tca6408a_config);
    ESP_LOGD("ioport_set_pin_level_through_tca6408a", "Set P%d to high.",
             pin & CH101_TCA_GET_PIN_MASK);
  }
  else
  {
    tca6408a_set_low(pin_reg, &tca6408a_config);
    ESP_LOGD("ioport_set_pin_level_through_tca6408a", "Set P%d to low.",
             pin & CH101_TCA_GET_PIN_MASK);
  }
}

/**
 * @brief Set the level of a general-purpose I/O pin.
 *
 * This function is the external API used for setting the level of both direct
 * GPIO pins and pins accessed through a TCA6408A I/O expander. It routes the
 * pin setting request to the appropriate handler based on the pin format.
 *
 * @param pin The pin to set the level of. Can be a direct GPIO or a TCA6408A
 * pin.
 * @param val The level to set the pin to, where 0 is low and 1 is high.
 *
 * @example ioport_set_pin_level(CH101_INT_TCA6408A_PIN, GPIO_LEVEL_HIGH); //
 * Set TCA6408A P1 to high
 * @example ioport_set_pin_level(GPIO_NUM_18, GPIO_LEVEL_LOW); // Set GPIO18 to
 * low
 */
static void ioport_set_pin_level(uint8_t pin, uint32_t val)
{
  /* special case for INT line */
  if (pin == CHIRP_INT_0)
  {
    /* No special case for changing level */
  }

  // if pin is CH101_TCA_PIN format (7th bit is 1)
  if (pin & CH101_TCA_PIN_MASK)
  {
    ioport_set_pin_level_through_tca6408a((ch101_tca6408a_pin_t)pin, val);
  }
  else
  {
    ioport_set_pin_level_through_gpio((gpio_num_t)pin, val);
  }
}

/* I2C related functions */

/* For CH-101 usage */
static void i2c_master0_init()
{
  const i2c_port_t i2c0_port = CHBSP_I2C_NUM;
  i2c_config_t i2c0_config = {.mode = I2C_MODE_MASTER,
                              .sda_io_num = GPIO_NUM_12,
                              .scl_io_num = GPIO_NUM_13,
                              /* TXS0102 requires not external pullup */
                              .sda_pullup_en = GPIO_PULLUP_DISABLE,
                              .scl_pullup_en = GPIO_PULLUP_DISABLE,
                              .master.clk_speed = I2C_MASTER_FREQ_HZ};

  ESP_ERROR_CHECK(i2c_param_config(i2c0_port, &i2c0_config));

  /* WARNING: we disable the i2c master interrupt (which samg55 enable) */
  /* See i2c_master_register_event_callbacks() for more information */
  ESP_ERROR_CHECK(i2c_driver_install(i2c0_port, i2c0_config.mode,
                                     I2C_MASTER_TX_BUF_DISABLE,
                                     I2C_MASTER_RX_BUF_DISABLE, 0));
}

/* For TCA6408A usage */
static void i2c_master1_init()
{
  const i2c_port_t i2c1_port = TCA6408A_I2C_NUM;
  i2c_config_t i2c1_config = {.mode = I2C_MODE_MASTER,
                              .sda_io_num = GPIO_NUM_4,
                              .scl_io_num = GPIO_NUM_5,

                              /* already pull up by external resistors */
                              .sda_pullup_en = GPIO_PULLUP_DISABLE,
                              .scl_pullup_en = GPIO_PULLUP_DISABLE,
                              .master.clk_speed = I2C_MASTER_FREQ_HZ};

  ESP_ERROR_CHECK(i2c_param_config(i2c1_port, &i2c1_config));

  /* WARNING: we disable the i2c master interrupt (which samg55 enable) */
  /* See i2c_master_register_event_callbacks() for more information */
  ESP_ERROR_CHECK(i2c_driver_install(i2c1_port, i2c1_config.mode,
                                     I2C_MASTER_TX_BUF_DISABLE,
                                     I2C_MASTER_RX_BUF_DISABLE, 0));
}

esp_err_t i2c_master0_write_register(uint8_t address, uint8_t register_address,
                                     size_t len, uint8_t* register_value)
{
  i2c_port_t i2c_num = CHBSP_I2C_NUM;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, register_address, true);
  i2c_master_write(cmd, register_value, len, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t i2c_master0_write_register_raw(unsigned char address,
                                         unsigned short len,
                                         unsigned char* data)
{
  const i2c_port_t i2c_num = CHBSP_I2C_NUM;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write(cmd, data, len, true);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);

  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t i2c_master0_read_register(uint8_t address, uint8_t register_address,
                                    uint16_t register_len,
                                    uint8_t* register_val)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  /* start and info slave which internal reg address will be operated (write
   * before read) */

  i2c_port_t i2c_num = CHBSP_I2C_NUM;

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, register_address, true);

  /* read back from slave */
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
  if (register_len > 1)
  {
    i2c_master_read(cmd, register_val, register_len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, register_val + register_len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);

  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

esp_err_t i2c_master0_read_register_raw(uint8_t address, size_t len,
                                        uint8_t* data)
{
  i2c_port_t i2c_num = CHBSP_I2C_NUM;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
  if (len > 1)
  {
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  return ret;
}

static void find_sensors(void)
{
  uint8_t sig_bytes[2];

  /* WARNING: RST is same pin for test config, while in formal project HAND
   * might not */
  ioport_set_pin_dir(CHIRP_RST, GPIO_MODE_OUTPUT);
  ioport_set_pin_level(CHIRP_RST, GPIO_LEVEL_HIGH);

  /* Drive PROG low on all sensor ports */
  /* TODO: make this a loop (limited by CHIRP_MAX_NUM_SENSORS) */
  ioport_set_pin_dir(CHIRP_PROG_0, GPIO_MODE_OUTPUT);  // PROG_0=output
  // ioport_set_pin_dir(CHIRP_PROG_1, GPIO_MODE_OUTPUT);  // PROG_1=output
  // ioport_set_pin_dir(CHIRP_PROG_2, GPIO_MODE_OUTPUT);  // PROG_2=output
  // ioport_set_pin_dir(CHIRP_PROG_3, GPIO_MODE_OUTPUT);  // PROG_3=output
  ioport_set_pin_level(CHIRP_PROG_0, GPIO_LEVEL_LOW);  // PROG_0=L
  // ioport_set_pin_level(CHIRP_PROG_1, GPIO_LEVEL_LOW);  // PROG_1=L
  // ioport_set_pin_level(CHIRP_PROG_2, GPIO_LEVEL_LOW);  // PROG_2=L
  // ioport_set_pin_level(CHIRP_PROG_3, GPIO_LEVEL_LOW);  // PROG_3=L

  /* check sensor 0 */
  /* WARNING: there's only 1 CH101 for test config */
  ioport_set_pin_level(CHIRP_PROG_0, GPIO_LEVEL_HIGH);
  tca6408a_test_read(&tca6408a_config);

  /* WARNING: move to ext_int_init, because esp-idf i2c driver requires delete
   * before reinstall. Not like samg55. */
  // i2c_master0_init();

  sig_bytes[0] = 0;
  sig_bytes[1] = 0;
  esp_err_t err =
      i2c_master0_read_register(CH_I2C_ADDR_PROG, 0x00, 2, sig_bytes);
  if (err != ESP_OK)
  {
    ESP_LOGE("find_sensors", "CH-101 I2C bus err: %d", err);
  }

  int find_sensor_flag =
      (sig_bytes[0] == CH_SIG_BYTE_0) && (sig_bytes[1] == CH_SIG_BYTE_1);
  ESP_LOGI("find_sensors", "Chirp sensor 0: %s.",
           find_sensor_flag ? " FOUND " : "NOT FOUND");
  ioport_set_pin_level(CHIRP_PROG_0, GPIO_LEVEL_LOW);
}

static void IRAM_ATTR chirp_isr_callback(void* ch_io_cb_para)
{
  /* Do not use ESP_LOG in this function */
  ch_io_int_callback_parameter_t* para =
      (ch_io_int_callback_parameter_t*)ch_io_cb_para;
  if (para->grp_ptr->io_int_callback != NULL)
  {
    ch_io_int_callback_t func_ptr = para->grp_ptr->io_int_callback;
    func_ptr(para->grp_ptr, para->io_index);
  }
}

/* interrupt config */
void ext_int_init(void)
{
  /* config an interrupt pin (IO 6) with ISR and pull down */
  /* default to input type */
  gpio_config_t int_config = {.intr_type = GPIO_INTR_POSEDGE,
                              .mode = GPIO_MODE_INPUT,
                              .pin_bit_mask = (1ULL << CHIRP_INT_IO6),
                              .pull_down_en = GPIO_PULLDOWN_ENABLE,
                              .pull_up_en = GPIO_PULLUP_DISABLE};

  gpio_config(&int_config);

  /* add interrupt and it's callback function */
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM);
  gpio_isr_handler_add(CHIRP_INT_IO6, NULL, NULL);
  gpio_intr_disable(CHIRP_INT_IO6);
}

/*!
 * \brief Initialize board hardware
 *
 * \note This function performs all necessary initialization on the board.
 */
void chbsp_board_init(ch_group_t* grp_ptr)
{
  /* Make local copy of group pointer */
  sensor_group_ptr = grp_ptr;

  /* Initialize group descriptor */
  grp_ptr->num_ports = CHIRP_USE_NUM_SENSORS;
  grp_ptr->num_i2c_buses = CHBSP_NUM_I2C_BUSES;
  grp_ptr->rtc_cal_pulse_ms = CHBSP_RTC_CAL_PULSE_MS;

  /* ESP-S3 init */
  ESP_LOGI("chbsp_board_init", "Chbsp board init procedure starts!");
  ext_int_init();
  chbsp_i2c_init();
  find_sensors();

  /* TODO: indicate alive */
  // indicate_alive();
}

/*!
 * \brief Assert the reset pin
 *
 * This function drives the sensor reset pin low.
 */
void chbsp_reset_assert(void)
{
  ioport_set_pin_level(CHIRP_RST, GPIO_LEVEL_LOW);  // reset=L
}

/*!
 * \brief Deassert the reset pin
 *
 * This function drives the sensor reset pin high.
 */
void chbsp_reset_release(void)
{
  ioport_set_pin_level(CHIRP_RST, GPIO_LEVEL_HIGH);  // reset=H
}

/*!
 * \brief Assert the PROG pin
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a
 * sensor
 *
 * This function drives the sensor PROG pin high on the specified port.
 */
void chbsp_program_enable(ch_dev_t* dev_ptr)
{
  uint8_t dev_num = ch_get_dev_num(dev_ptr);

  /* for HAND_BUILD_TARGET_PORTING_CH101, we need to make sure that dev_num
   * always be 0 */
  if (dev_num != 0)
  {
    ESP_LOGE("chbsp_program_enable",
             "Dev number should always be 0 under "
             "HAND_BUILD_TARGET_PORTING_CH101. But now it's: %d. Abort!",
             dev_num);
    return;
  }

  // select Chirp chip PROGRAM line on Atmel board according to chip number
  ioport_set_pin_level(chirp_pin_prog[dev_num],
                       GPIO_LEVEL_HIGH);  // PROG_0=H
}

/*!
 * \brief Deassert the PROG pin
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a
 * sensor
 *
 * This function drives the sensor PROG pin low on the specified port.
 */
void chbsp_program_disable(ch_dev_t* dev_ptr)
{
  uint8_t dev_num = ch_get_dev_num(dev_ptr);

  /* for HAND_BUILD_TARGET_PORTING_CH101, we need to make sure that dev_num
   * always be 0 */
  if (dev_num != 0)
  {
    ESP_LOGE("chbsp_program_disable",
             "Dev number should always be 0 under "
             "HAND_BUILD_TARGET_PORTING_CH101. But now it's: %d. Abort!",
             dev_num);
    return;
  }

  // select Chirp chip PROGRAM line on Atmel board according to chip number
  ioport_set_pin_level(chirp_pin_prog[dev_num],
                       GPIO_LEVEL_LOW);  // PROG_0=L
}

/*!
 * \brief Configure the Chirp sensor INT pin as an output for one sensor.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a
 * sensor
 *
 * This function configures the Chirp sensor INT pin as an output (from the
 * perspective of the host system).
 */
void chbsp_set_io_dir_out(ch_dev_t* dev_ptr)
{
  uint8_t dev_num = ch_get_dev_num(dev_ptr);

  /* for HAND_BUILD_TARGET_PORTING_CH101, we need to make sure that dev_num
   * always be 0 */
  if (dev_num != 0)
  {
    ESP_LOGE("chbsp_set_io_dir_out",
             "Dev number should always be 0 under "
             "HAND_BUILD_TARGET_PORTING_CH101. But now it's: %d. Abort!",
             dev_num);
    return;
  }

  ioport_set_pin_dir(chirp_pin_io[dev_num], GPIO_MODE_OUTPUT);
}

/*!
 * \brief Configure the Chirp sensor INT pin as an input for one sensor.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a
 * sensor
 *
 * This function configures the Chirp sensor INT pin as an input (from the
 * perspective of the host system).
 */
void chbsp_set_io_dir_in(ch_dev_t* dev_ptr)
{
  uint8_t dev_num = ch_get_dev_num(dev_ptr);

  if (dev_num != 0)
  {
    ESP_LOGE("chbsp_set_io_dir_in",
             "Dev number should always be 0 under "
             "HAND_BUILD_TARGET_PORTING_CH101. But now it's: %d. Abort!",
             dev_num);
    return;
  }

  ioport_set_pin_dir(chirp_pin_io[dev_num], GPIO_MODE_INPUT);
}

/*!
 * \brief Configure the Chirp sensor INT pins as outputs for a group of sensors
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a
 * group of sensors
 *
 * This function configures each Chirp sensor's INT pin as an output (from the
 * perspective of the host system).
 */
void chbsp_group_set_io_dir_out(ch_group_t* grp_ptr)
{
  uint8_t dev_num;

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
  {
    ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    /* TODO: to improve performance, Please take a look at chbsp_chirp_samg55.c
     */
    /* Note: there's no differene in performance if you only have one CH-101 */
    if (ch_sensor_is_connected(dev_ptr))
    {
      chbsp_set_io_dir_out(dev_ptr);
    }
  }
}

/*!
 * \brief Configure the Chirp sensor INT pins as inputs for a group of sensors
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a
 * group of sensors
 *
 * \note This function assumes a bidirectional level shifter is interfacing the
 * ICs.
 */
void chbsp_group_set_io_dir_in(ch_group_t* grp_ptr)
{
  uint8_t dev_num;

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
  {
    ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    /* TODO: to improve performance, Please take a look at chbsp_chirp_samg55.c
     */
    /* Note: there's no differene in performance if you only have one CH-101 */
    if (ch_sensor_is_connected(dev_ptr))
    {
      chbsp_set_io_dir_in(dev_ptr);
    }
  }
}

/*!
 * \brief Initialize the I/O pins.
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a
 * group of sensors
 *
 * Configure reset and program pins as outputs. Assert reset and program.
 * Configure sensor INT pin as input.
 */
void chbsp_group_pin_init(ch_group_t* grp_ptr)
{
  uint8_t dev_num;

  ioport_set_pin_dir(CHIRP_PROG_0, GPIO_MODE_OUTPUT);  // PROG_0=output
  // ioport_set_pin_dir(CHIRP_PROG_1, IOPORT_DIR_OUTPUT); //PROG_1=output
  // ioport_set_pin_dir(CHIRP_PROG_2, IOPORT_DIR_OUTPUT); //PROG_2=output
  // ioport_set_pin_dir(CHIRP_PROG_3, IOPORT_DIR_OUTPUT); //PROG_3=output

  ioport_set_pin_level(CHIRP_PROG_0, GPIO_LEVEL_LOW);  // PROG_0=L
  // ioport_set_pin_level(CHIRP_PROG_1, IOPORT_PIN_LEVEL_LOW); //PROG_1=L
  // ioport_set_pin_level(CHIRP_PROG_2, IOPORT_PIN_LEVEL_LOW); //PROG_2=L
  // ioport_set_pin_level(CHIRP_PROG_3, IOPORT_PIN_LEVEL_LOW); //PROG_3=L

  ioport_set_pin_dir(CHIRP_RST, GPIO_MODE_OUTPUT);  // reset=output
  chbsp_reset_assert();

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
  {
    ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    chbsp_program_enable(dev_ptr);
  }

  /* Initialize IO (INT) pins */
  /* Note: including intr enable internal */
  chbsp_group_set_io_dir_in(grp_ptr);

  /* Configure PIOs interrupt as input pins. */
  /* WARNING: we assume interrupt related flags already be set */
  ESP_LOGW("chbsp_group_pin_init",
           "We assume the interrupt config was done in ext_int_init and the "
           "related flags never be changed!");
  // for(port_num = 0; port_num < grp_ptr->num_ports; port_num++ ) {
  // 	pio_configure(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_TYPE,
  // chirp_pin_io_irq[port_num], PIN_EXT_INTERRUPT_ATTR);
  // }

  /* remove motionINT */
  // pio_configure(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_TYPE,
  // PIN_EXT_MotionINT_MASK, 		      PIN_EXT_INTERRUPT_ATTR);

  /* Initialize PIO interrupt handler, interrupt on rising edge. */
  /* Note: gpio_install_isr_service() already called in ext_int_init() */
  /* Note: do not create ch_io_it_cb_para0 in this function, which leads to pass
   * stack-allocated pointers to ISRs problem */
  /* create a callback for sensor 0 */
  ch_io_it_cb_para0.grp_ptr = grp_ptr;  // update grp_ptr
  ch_io_it_cb_para0.io_index = 0;       // sensor 0
  gpio_isr_handler_add(CHIRP_INT_IO6, chirp_isr_callback,
                       (void*)&ch_io_it_cb_para0);

  // TODO: other handler for sensor 1, 2, ...
  // gpio_isr_handler_add(CHIRP_IO_2, chirp_isr_callback,
  // (void*)&para1);

  /* remove motionINT handler */
  // pio_handler_set(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_ID,
  // PIN_EXT_MotionINT_MASK,
  // PIN_EXT_INTERRUPT_ATTR, (void (*) (uint32_t,
  // uint32_t))ext_MotionINT_handler);

  /* remove push button (PIO) interrupt. */
  // pio_handler_set_priority(PIN_EXT_INTERRUPT_PIO, PIN_EXT_INTERRUPT_IRQn, 0);
}

/*!
 * \brief Set the INT pins low for a group of sensors.
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a
 * group of sensors
 *
 * This function drives the INT line low for each sensor in the group.
 */
void chbsp_group_io_clear(ch_group_t* grp_ptr)
{
  uint8_t dev_num;

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
  {
    ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    if (ch_sensor_is_connected(dev_ptr))
    {
      /* TODO: to improve performance, Please take a look at
       * chbsp_chirp_samg55.c
       */
      /* Note: there's no differene in performance if you only have one CH-101
       */
      chbsp_io_clear(dev_ptr);
    }
  }
}

/*!
 * \brief Set the INT pins high for a group of sensors.
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a
 * group of sensors
 *
 * This function drives the INT line high for each sensor in the group.
 */
void chbsp_group_io_set(ch_group_t* grp_ptr)
{
  uint8_t dev_num;

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
  {
    ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    if (ch_sensor_is_connected(dev_ptr))
    {
      /* TODO: to improve performance, Please take a look at
       * chbsp_chirp_samg55.c
       */
      /* Note: there's no differene in performance if you only have one CH-101
       */
      chbsp_io_set(dev_ptr);
    }
  }
}

/*!
 * \brief Disable interrupts for a group of sensors
 *
 * \param grp_ptr 	pointer to the ch_group_t config structure for a group
 * of sensors
 *
 * For each sensor in the group, this function disables the host interrupt
 * associated with the Chirp sensor device's INT line.
 */
void chbsp_group_io_interrupt_enable(ch_group_t* grp_ptr)
{
  uint8_t dev_num;

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
  {
    ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    chbsp_io_interrupt_enable(dev_ptr);
  }
}

/*!
 * \brief Enable the interrupt for one sensor
 *
 * \param dev_ptr	pointer to the ch_dev_t config structure for a sensor
 *
 * This function enables the host interrupt associated with the Chirp sensor
 * device's INT line.
 */
void chbsp_io_interrupt_enable(ch_dev_t* dev_ptr)
{
  uint8_t dev_num = ch_get_dev_num(dev_ptr);

  if (dev_num != 0)
  {
    ESP_LOGE("chbsp_io_interrupt_enable",
             "Dev number should always be 0 under "
             "HAND_BUILD_TARGET_PORTING_CH101. But now it's: %d. Abort!",
             dev_num);
    return;
  }

  if (ch_sensor_is_connected(dev_ptr))
  {
    gpio_intr_enable(CHIRP_INT_IO6);
  }
}

/*!
 * \brief Disable interrupts for a group of sensors
 *
 * \param grp_ptr 		pointer to the ch_group_t config structure for a
 * group of sensors
 *
 * For each sensor in the group, this function disables the host interrupt
 * associated with the Chirp sensor device's INT line.
 */
void chbsp_group_io_interrupt_disable(ch_group_t* grp_ptr)
{
  uint8_t dev_num;

  for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
  {
    ch_dev_t* dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

    chbsp_io_interrupt_disable(dev_ptr);
  }
}

/*!
 * \brief Disable the interrupt for one sensor
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a
 * sensor
 *
 * This function disables the host interrupt associated with the Chirp sensor
 * device's INT line.
 */
void chbsp_io_interrupt_disable(ch_dev_t* dev_ptr)
{
  uint8_t dev_num = ch_get_dev_num(dev_ptr);

  if (dev_num != 0)
  {
    ESP_LOGE("chbsp_io_interrupt_disable",
             "Dev number should always be 0 under "
             "HAND_BUILD_TARGET_PORTING_CH101. But now it's: %d. Abort!",
             dev_num);
    return;
  }

  if (ch_sensor_is_connected(dev_ptr))
  {
    gpio_intr_disable(CHIRP_INT_IO6);
  }
}

/*!
 * \brief Set the INT pins low for a one sensor.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a
 * sensor
 *
 * This function drives the INT line low for one sensor.
 */
void chbsp_io_clear(ch_dev_t* dev_ptr)
{
  uint8_t dev_num = ch_get_dev_num(dev_ptr);

  if (dev_num != 0)
  {
    ESP_LOGE("chbsp_io_clear",
             "Dev number should always be 0 under "
             "HAND_BUILD_TARGET_PORTING_CH101. But now it's: %d. Abort!",
             dev_num);
    return;
  }

  if (ch_sensor_is_connected(dev_ptr))
  {
    ioport_set_pin_level(chirp_pin_io[dev_num], GPIO_LEVEL_LOW);
  }
}

/*!
 * \brief Set the INT pins high for a one sensor.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a
 * sensor
 *
 * This function drives the INT line high for one sensor.
 */
void chbsp_io_set(ch_dev_t* dev_ptr)
{
  uint8_t dev_num = ch_get_dev_num(dev_ptr);

  if (dev_num != 0)
  {
    ESP_LOGE("chbsp_io_set",
             "Dev number should always be 0 under "
             "HAND_BUILD_TARGET_PORTING_CH101. But now it's: %d. Abort!",
             dev_num);
    return;
  }

  if (ch_sensor_is_connected(dev_ptr))
  {
    ioport_set_pin_level(chirp_pin_io[dev_num], GPIO_LEVEL_HIGH);
  }
}

/*!
 * \brief Set callback routine for Chirp sensor I/O interrupt
 *
 * \param callback_func_ptr 	pointer to application function to be called
 * when interrupt occurs
 *
 * This function sets up the specified callback routine to be called whenever
 * the interrupt associated with the sensor's INT line occurs.  The callback
 * routine address in stored in a pointer variable that will later be accessed
 * from within the interrupt handler to call the function.
 *
 * The callback function will be called at interrupt level from the interrupt
 * service routine.
 */
void chbsp_io_callback_set(ch_io_int_callback_t callback_func_ptr)
{
  /* for single usage, comment this if not use */
  // io_int_callback_ptr = callback_func_ptr;
}

/**
 * @brief Delay for specified number of microseconds.
 *
 * @param us  number of microseconds to delay before returning
 *
 * This function waits for the specified number of microseconds before returning
 * to the caller.
 */
void chbsp_delay_us(uint32_t us)
{
  /* WARNING: it's a unstable api of esp-idf */
  /* TODO: improve this later */
  esp_rom_delay_us(us);
}

/**
 * @brief Delay for specified number of milliseconds.
 *
 * @param ms  number of milliseconds to delay before returning
 *
 * This function waits for the specified number of milliseconds before returning
 * to the caller.
 */
void chbsp_delay_ms(uint32_t ms)
{
  if (ms < 100)
  {
    // TODO: anyway to wait in async?
    chbsp_delay_us(ms * 1000);
  }
  else
  {
    vTaskDelay(pdMS_TO_TICKS(ms));
  }
}

/**
 * @brief Initialize the host's I2C hardware.
 *
 * @return 0 if successful, 1 on error
 *
 * This function performs general I2C initialization on the host system.
 */
int chbsp_i2c_init(void)
{
  static int is_init = 0;
  if (!is_init)
  {
    i2c_master0_init();
    i2c_master1_init();
  }
  else
  {
    ESP_LOGW("chbsp_i2c_init", "I2C buses were already initialized!");
  }
  is_init = 1;
  return 0;
}

/**
 * @brief Return I2C information for a sensor port on the board.
 *
 * @param grp_ptr    pointer to the ch_group_t config structure for a group of
 * sensors
 * @param dev_num    device number within sensor group
 * @param info_ptr   pointer to structure to be filled with I2C config values
 *
 * @return 0 if successful, 1 if error
 *
 * This function returns I2C values in the ch_i2c_info_t structure specified by
 * @a info_ptr. The structure includes three fields.
 *  - The @a address field contains the I2C address for the sensor.
 *  - The @a bus_num field contains the I2C bus number (index).
 *  - The @a drv_flags field contains various bit flags through which the BSP
 * can inform SonicLib driver functions to perform specific actions during I2C
 * I/O operations.
 */
uint8_t chbsp_i2c_get_info(ch_group_t __attribute__((unused)) * grp_ptr,
                           uint8_t io_index, ch_i2c_info_t* info_ptr)
{
  uint8_t ret_val = 1;

  if (io_index <= CHBSP_MAX_DEVICES)
  {
    info_ptr->address = chirp_i2c_addrs[io_index];
    info_ptr->bus_num = chirp_i2c_buses[io_index];

    info_ptr->drv_flags =
        0;  // no special I2C handling by SonicLib driver is needed

    ret_val = 0;
  }

  return ret_val;
}

/**
 * @brief Write bytes to an I2C slave.
 *
 * @param dev_ptr    pointer to the ch_dev_t config structure for a sensor
 * @param data       data to be transmitted
 * @param num_bytes  length of data to be transmitted
 *
 * @return 0 if successful, 1 on error or NACK
 *
 * This function writes one or more bytes of data to an I2C slave device.
 * The I2C interface must have already been initialized using @a
 * chbsp_i2c_init().
 */
esp_err_t chbsp_i2c_write(ch_dev_t* dev_ptr, uint8_t* data, uint16_t num_bytes)
{
  esp_err_t error = 0;

  if (dev_ptr->i2c_bus_index == 0)
  {
    error =
        i2c_master0_write_register_raw(dev_ptr->i2c_address, num_bytes, data);
  }
  else if (dev_ptr->i2c_bus_index == 1)
  {
    ESP_LOGE("chbsp_i2c_write", "I2C bus should always be 0.");
  }

  return error;
}

/**
 * @brief Write bytes to an I2C slave using memory addressing.
 *
 * @param dev_ptr    pointer to the ch_dev_t config structure for a sensor
 * @param mem_addr   internal memory or register address within device
 * @param data       data to be transmitted
 * @param num_bytes  length of data to be transmitted
 *
 * @return 0 if successful, 1 on error or NACK
 *
 * This function writes one or more bytes of data to an I2C slave device using
 * an internal memory or register address. The remote device will write @a
 * num_bytes bytes of data starting at internal memory/register address @a
 * mem_addr. The I2C interface must have already been initialized using @a
 * chbsp_i2c_init().
 */
esp_err_t chbsp_i2c_mem_write(ch_dev_t* dev_ptr, uint16_t mem_addr,
                              uint8_t* data, uint16_t num_bytes)
{
  esp_err_t error = 0;
  if (dev_ptr->i2c_bus_index == 0)
  {
    error = i2c_master0_write_register(dev_ptr->i2c_address, mem_addr,
                                       num_bytes, data);
  }
  else if (dev_ptr->i2c_bus_index == 1)
  {
    ESP_LOGE("chbsp_i2c_mem_write", "I2C bus should always be 0.");
  }
  return error;
}

/**
 * @brief Write bytes to an I2C slave, non-blocking.
 *
 * @param dev_ptr    pointer to the ch_dev_t config structure for a sensor
 * @param data       pointer to the start of data to be transmitted
 * @param num_bytes  length of data to be transmitted
 *
 * @return 0 if successful, 1 on error or NACK
 *
 * This function initiates a non-blocking write of the specified number of bytes
 * to an I2C slave device.
 *
 * The I2C interface must have already been initialized using @a
 * chbsp_i2c_init().
 */
int chbsp_i2c_write_nb(ch_dev_t __attribute__((unused)) * dev_ptr,
                       uint8_t __attribute__((unused)) * data,
                       uint16_t __attribute__((unused)) num_bytes)
{
  // TODO: implement for HAND_BUILD_TARGET_PORTING_CH101 later
  ESP_LOGW("chbsp_i2c_write_nb",
           "Function: chbsp_i2c_write_nb not implemented!");
  return 1;
}

/**
 * @brief Write bytes to an I2C slave using memory addressing, non-blocking.
 *
 * @param dev_ptr    pointer to the ch_dev_t config structure for a sensor
 * @param mem_addr   internal memory or register address within device
 * @param data       pointer to the start of data to be transmitted
 * @param num_bytes  length of data to be transmitted
 *
 * @return 0 if successful, 1 on error or NACK
 *
 * This function initiates a non-blocking write of the specified number of bytes
 * to an I2C slave device, using an internal memory or register address. The
 * remote device will write @a num_bytes bytes of data starting at internal
 * memory/register address @a mem_addr.
 *
 * The I2C interface must have already been initialized using @a
 * chbsp_i2c_init().
 */
int chbsp_i2c_mem_write_nb(ch_dev_t __attribute__((unused)) * dev_ptr,
                           uint16_t __attribute__((unused)) mem_addr,
                           uint8_t __attribute__((unused)) * data,
                           uint16_t __attribute__((unused)) num_bytes)
{
  // TODO: implement for HAND_BUILD_TARGET_PORTING_CH101 later
  ESP_LOGW("chbsp_i2c_mem_write_nb",
           "Function: chbsp_i2c_mem_write_nb not implemented!");
  return 1;
}

/**
 * @brief Read bytes from an I2C slave.
 *
 * @param dev_ptr    pointer to the ch_dev_t config structure for a sensor
 * @param data       pointer to receive data buffer
 * @param num_bytes  number of bytes to read
 *
 * @return 0 if successful, 1 on error or NACK
 *
 * This function reads the specified number of bytes from an I2C slave device.
 * The I2C interface must have already been initialized using @a
 * chbsp_i2c_init().
 */
esp_err_t chbsp_i2c_read(ch_dev_t* dev_ptr, uint8_t* data, uint16_t num_bytes)
{
  esp_err_t error = 1;
  uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
  uint8_t bus_num = ch_get_i2c_bus(dev_ptr);

  if (bus_num == 0)
  {
    error = i2c_master0_read_register_raw(i2c_addr, num_bytes, data);
  }
  else if (bus_num == 1)
  {
    ESP_LOGE("chbsp_i2c_read", "I2C bus should always be 0.");
  }
  return error;
}

/**
 * @brief Read bytes from an I2C slave using memory addressing.
 *
 * @param dev_ptr      pointer to the ch_dev_t config structure for a sensor
 * @param mem_addr     internal memory or register address within device
 * @param data         pointer to receive data buffer
 * @param num_bytes    number of bytes to read
 *
 * @return 0 if successful, 1 on error or NACK
 *
 * This function reads the specified number of bytes from an I2C slave device,
 * using an internal memory or register address.  The remote device will return
 * @a num_bytes bytes starting at internal memory/register address @a mem_addr.
 *
 * The I2C interface must have already been initialized using @a
 * chbsp_i2c_init().
 */
esp_err_t chbsp_i2c_mem_read(ch_dev_t* dev_ptr, uint16_t mem_addr,
                             uint8_t* data, uint16_t num_bytes)
{
  esp_err_t error = 1;
  uint8_t i2c_addr = ch_get_i2c_address(dev_ptr);
  uint8_t bus_num = ch_get_i2c_bus(dev_ptr);

  if (bus_num == 0)
  {
    error = i2c_master0_read_register(i2c_addr, mem_addr, num_bytes, data);
  }
  else if (bus_num == 1)
  {
    ESP_LOGE("chbsp_i2c_read", "I2C bus should always be 0.");
  }
  return error;
}

/*!
 * \brief Read bytes from an I2C slave, non-blocking.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a
 * sensor \param data 			pointer to receive data buffer \param
 * num_bytes 	number of bytes to read
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function initiates a non-blocking read of the specified number of bytes
 * from an I2C slave.
 *
 * The I2C interface must have already been initialized using \a
 * chbsp_i2c_init().
 */
int chbsp_i2c_read_nb(ch_dev_t* dev_ptr, uint8_t* data, uint16_t num_bytes)
{
  /* TODO: implement later */
  ESP_LOGW("chbsp_i2c_read_nb", "Function: chbsp_i2c_read_nb not implemented!");
  return 1;
}

/*!
 * \brief Read bytes from an I2C slave using memory addressing, non-blocking.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a
 * sensor \param mem_addr		internal memory or register address
 * within device \param data 			pointer to receive data buffer
 * \param num_bytes 	number of bytes to read
 *
 * \return 0 if successful, 1 on error or NACK
 *
 * This function initiates a non-blocking read of the specified number of bytes
 * from an I2C slave.
 *
 * The I2C interface must have already been initialized using \a
 * chbsp_i2c_init().
 */
int chbsp_i2c_mem_read_nb(ch_dev_t* dev_ptr, uint16_t mem_addr, uint8_t* data,
                          uint16_t num_bytes)
{
  ESP_LOGW("chbsp_i2c_mem_read_nb",
           "Function: chbsp_i2c_mem_read_nb not implemented!");
  return 1;
}

/*!
 * \brief Reset I2C bus associated with device.
 *
 * \param dev_ptr 		pointer to the ch_dev_t config structure for a
 * sensor
 *
 * This function performs a reset of the I2C interface for the specified device.
 */
void chbsp_i2c_reset(ch_dev_t* dev_ptr)
{
  uint8_t bus_num = ch_get_i2c_bus(dev_ptr);

  if (bus_num == 0)
  {
    /* TODO: check need deinit() here or not */
    i2c_master0_init();
  }
  else if (bus_num == 1)
  {
    ESP_LOGE("chbsp_i2c_reset", "I2C bus should always be 0!");
  }
}

/*!
 * \brief Initialize periodic timer.
 *
 * \param interval_ms		timer interval, in milliseconds
 * \param callback_func_ptr	address of routine to be called every time the
 * timer expires
 *
 * \return 0 if successful, 1 if error
 *
 * This function initializes a periodic timer on the board.  The timer is
 * programmed to generate an interrupt after every \a interval_ms milliseconds.
 *
 * The \a callback_func_ptr parameter specifies a callback routine that will be
 * called when the timer expires (and interrupt occurs).  The \a
 * chbsp_periodic_timer_handler function will call this function.
 */
uint8_t chbsp_periodic_timer_init(uint16_t interval_ms,
                                  ch_timer_callback_t callback_func_ptr)
{
  static bool is_hw_init_done = false;

  /* Save timer interval and callback function */
  periodic_timer_interval_us = interval_ms * 1000;
  periodic_timer_callback_ptr = callback_func_ptr;

  /* Initialize the HW only 1 time at startup. Skip the init on subsequent
   * calls. */
  if (!is_hw_init_done)
  {
    const esp_timer_create_args_t timer_args = {
        .callback = chbsp_periodic_timer_handler,
        .name = "ch101_periodic_timer"};

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &periodic_timer_handle_ptr));
    ESP_LOGI("chbsp_periodic_timer_init", "Periodic timer init done!");
  }

  /* Mark the HW init as done */
  is_hw_init_done = true;

  return 0;
}

void chbsp_periodic_timer_change_period(uint32_t new_interval_us)
{
  /* TODO: could we use esp_timer_restart(handle, new_interval_us) */
  periodic_timer_interval_us = new_interval_us;
  /* restart if timer is running */
  if (esp_timer_is_active(periodic_timer_handle_ptr))
  {
    esp_timer_restart(periodic_timer_handle_ptr, periodic_timer_interval_us);
  }
}

/* For esp-idf, this function is not needed */

// uint32_t get_period_in_tick(uint32_t interval_us)
// {
//   uint64_t timer_period_in_tick =
//       (uint64_t)ULTRASOUND_TIMER_FREQUENCY * interval_us / 1000000;

//   /* If the ODR is too slow to be handled then program a faster interrupt and
//    * decimate it */
//   if (timer_period_in_tick > UINT16_MAX)
//     decimation_factor = timer_period_in_tick / UINT16_MAX + 1;
//   else
//     decimation_factor = 1;

//   /* Calculate the final tick in case a decimation is needed */
//   return (uint32_t)(timer_period_in_tick / decimation_factor);
// }

/* For esp-idf, this function is not needed */

// void program_next_period(void)
// {
//   uint32_t time =
//       ultrasound_prev_period_end_in_tick + ultrasound_timer_period_in_tick;
//   ultrasound_prev_period_end_in_tick = time;
//   tc_write_rc(TC0, TC_CHANNEL_US, (uint16_t)(time & 0xFFFF));
// }

/*!
 * \brief Enable periodic timer interrupt.
 *
 * This function enables the interrupt associated with the periodic timer
 * initialized by \a chbsp_periodic_timer_init().
 */
void chbsp_periodic_timer_irq_enable(void)
{
  if (periodic_timer_handle_ptr == NULL)
  {
    ESP_LOGE("chbsp_periodic_timer_irq_enable",
             "Periodic timer not init yet! Please call "
             "`chbsp_periodic_timer_init()` first.");
  }

  if (esp_timer_is_active(periodic_timer_handle_ptr))
  {
    /* Do nothing */
  }
  else
  {
    esp_timer_start_periodic(periodic_timer_handle_ptr,
                             periodic_timer_interval_us);
  }
}

/*!
 * \brief Disable periodic timer interrupt.
 *
 * This function enables the interrupt associated with the periodic timer
 * initialized by \a chbsp_periodic_timer_init().
 */
void chbsp_periodic_timer_irq_disable(void)
{
  if (periodic_timer_handle_ptr == NULL)
  {
    ESP_LOGE("chbsp_periodic_timer_irq_disable",
             "Periodic timer not init yet! Please call "
             "`chbsp_periodic_timer_init()` first.");
  }

  /* esp-idf only has esp_timer_stop() */
  if (esp_timer_is_active(periodic_timer_handle_ptr))
  {
    esp_timer_stop(periodic_timer_handle_ptr);
  }
}

/*!
 * \brief Start periodic timer.
 *
 * \return 0 if successful, 1 if error
 *
 * This function starts the periodic timer initialized by \a
 * chbsp_periodic_timer_init().
 */
uint8_t chbsp_periodic_timer_start(void)
{
  if (esp_timer_is_active(periodic_timer_handle_ptr))
  {
    esp_timer_restart(periodic_timer_handle_ptr, periodic_timer_interval_us);
  }
  else
  {
    esp_timer_start_periodic(periodic_timer_handle_ptr,
                             periodic_timer_interval_us);
  }
  ESP_LOGW("chbsp_periodic_timer_start", "CH-101 periodic timer start.");
  return 0;
}

/*!
 * \brief Stop periodic timer.
 *
 * \return 0 if successful, 1 if error
 *
 * This function stops the periodic timer initialized by \a
 * chbsp_periodic_timer_init().
 */
uint8_t chbsp_periodic_timer_stop(void)
{
  if (periodic_timer_handle_ptr == NULL)
  {
    ESP_LOGE("chbsp_periodic_timer_stop",
             "Periodic timer not init yet! Please call "
             "`chbsp_periodic_timer_init()` first.");
    return 0;
  }

  if (esp_timer_is_active(periodic_timer_handle_ptr))
  {
    esp_timer_stop(periodic_timer_handle_ptr);
    ESP_LOGW("chbsp_periodic_timer_stop", "CH-101 periodic timer stop.");
  }
  return 0;
}

/*!
 * \brief Periodic timer handler.
 *
 * \return 0 if successful, 1 if error
 *
 * This function handles the expiration of the periodic timer, re-arms it and
 * any associated interrupts for the next interval, and calls the callback
 * routine that was registered using \a chbsp_periodic_timer_init().
 */
void chbsp_periodic_timer_handler(void* __attribute__((unused)) para)
{
  ch_timer_callback_t func_ptr = periodic_timer_callback_ptr;

  decimation_counter++;
  /* esp-idf timer system doesn't need this */
  // program_next_period();
  if (decimation_counter >= decimation_factor)
  {
    decimation_counter = 0;
    if (func_ptr != NULL)
    {
      (*func_ptr)();  // call application timer callback routine
    }
  }
}

/*!
 * \brief Put the processor into low-power sleep state.
 *
 * This function puts the host processor (MCU) into a low-power sleep mode, to
 * conserve energy. The sleep state should be selected such that interrupts
 * associated with the I2C, external GPIO pins, and the periodic timer (if used)
 * are able to wake up the device.
 */
void chbsp_proc_sleep(void)
{ /* We don't need this for demo */
}

/*!
 * \brief Turn on an LED on the board.
 *
 * This function turns on an LED on the board.
 *
 * The \a dev_num parameter contains the device number of a specific sensor.
 * This routine will turn on the LED on the Chirp sensor daughterboard that is
 * next to the specified sensor.
 */
void chbsp_led_on(uint8_t led_num)
{
  gpio_set_level(chirp_led_pins[led_num], GPIO_LEVEL_HIGH);
}

/*!
 * \brief Turn off an LED on the board.
 *
 * This function turns off an LED on the board.
 *
 * The \a dev_num parameter contains the device number of a specific sensor.
 * This routine will turn off the LED on the Chirp sensor daughterboard that is
 * next to the specified sensor.
 */
void chbsp_led_off(uint8_t led_num)
{
  gpio_set_level(chirp_led_pins[led_num], GPIO_LEVEL_LOW);
}

/*!
 * \brief Toggles an LED on the board.
 *
 * This function toggles an LED on the board.
 *
 * The \a dev_num parameter contains the device number of a specific sensor.
 * This routine will toggles the LED on the Chirp sensor daughterboard that is
 * next to the specified sensor.
 */
void chbsp_led_toggle(uint8_t led_num)
{
  /* XXX: if we use GPIO_MODE_OUTPUT, then gpio_get_level() always return 0.
   * Hence we should modify the config pin for led gpio. Currently, the
   * alternative is to use a static state, for there's only one sensor. */
  static int led0_state = GPIO_LEVEL_LOW;
  // int state = gpio_get_level(chirp_led_pins[led_num]);

  if (led_num != 0)
  {
    ESP_LOGE("chbsp_led_toggle", "The `io_index` should always be 0.");
  }

  gpio_set_level(chirp_led_pins[led_num], !led0_state);
  led0_state = !led0_state;
}

/*!
 * \brief Output a text string via serial interface
 *
 * \param str	pointer to a string of characters to be output
 *
 * This function prints debug information to the console.
 */
void chbsp_print_str(char* str) { printf(str); }

/*!
 * \brief Return the current time in ms
 *
 * This function returns the system current time in ms.
 */
int64_t chbsp_timestamp_ms(void)
{
  int64_t time = esp_timer_get_time();
  return (time / 1000);
}