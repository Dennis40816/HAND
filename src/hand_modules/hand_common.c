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
#include "hand_global.h"
#include "hand_common.h"

#include "driver/spi_master.h"
#include "driver/usb_serial_jtag.h"

#ifndef HAND_DEFAULT_LOG_SERVER_IP
#define HAND_DEFAULT_LOG_SERVER_IP "192.168.1.17"
#endif

static const char* TAG = "HAND_COMMON";

/* TODO: NEED REFACTOR */
static void vl53l1x_init(VL53L1_DEV dev, bool calibration_en)
{
  /* ROI 4*4 middle*/
  VL53L1_UserRoi_t default_roi = {
      .TopLeftX = 6, .TopLeftY = 9, .BotRightX = 9, .BotRightY = 6};

  ESP_LOGW(TAG, "Init process starts, all return status should be: {%d}",
           VL53L1_ERROR_NONE);

  /* init */
  int status = VL53L1_WaitDeviceBooted(dev);
  ESP_LOGI(TAG, "Status of `VL53L1_WaitDeviceBooted` is: {%d}", status);
  status = VL53L1_DataInit(dev);
  ESP_LOGI(TAG, "Status of `VL53L1_DataInit` is: {%d}", status);
  status = VL53L1_StaticInit(dev);
  ESP_LOGI(TAG, "Status of `VL53L1_StaticInit` is: {%d}", status);

  /* set perset mode */
  status = VL53L1_SetPresetMode(dev, VL53L1_PRESETMODE_AUTONOMOUS);
  ESP_LOGI(TAG, "Status of `VL53L1_SetPresetMode` is: {%d}", status);

  /* run calibration if enable */
  if (calibration_en)
  {
    /* TODO: run calibration */
  }

  status = VL53L1_SetDistanceMode(dev, VL53L1_DISTANCEMODE_SHORT);
  ESP_LOGI(TAG, "Status of `VL53L1_SetDistanceMode` is: {%d}", status);
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(
      dev, HAND_VL53L1X_DEFAULT_TIMING_BUDGET_MS * 1000);
  ESP_LOGI(TAG,
           "Status of `VL53L1_SetMeasurementTimingBudgetMicroSeconds` is: {%d}",
           status);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(
      dev, HAND_VL53L1X_DEFAULT_MEASURE_PERIOD_MS);
  ESP_LOGI(TAG,
           "Status of `VL53L1_SetInterMeasurementPeriodMilliSeconds` is: {%d}",
           status);

  status = VL53L1_SetUserROI(dev, &default_roi);
  ESP_LOGI(TAG, "Status of `VL53L1_SetUserROI` is: {%d}", status);

  ESP_LOGW(TAG, "Init process end");
}

static esp_err_t hand_i2c_bus_and_device_init(
    hand_devices_handle_t* devs_handle_p)
{
  esp_err_t ret = ESP_OK;

  /* init I2C_0 (I2C OTHER) */
  i2c_port_t i2c0_port = I2C_NUM_0;
  i2c_config_t i2c0_cfg = {.mode = I2C_MODE_MASTER,
                           .sda_io_num = HAND_PIN_I2C_OTHER_SDA,
                           .scl_io_num = HAND_PIN_I2C_OTHER_SCL,
                           /* TXS0102 requires not external pullup */
                           .sda_pullup_en = GPIO_PULLUP_DISABLE,
                           .scl_pullup_en = GPIO_PULLUP_DISABLE,
                           .master.clk_speed = HAND_BUS_I2C_0_SPEED};
  ESP_ERROR_CHECK(i2c_param_config(i2c0_port, &i2c0_cfg));
  ESP_ERROR_CHECK(i2c_driver_install(i2c0_port, i2c0_cfg.mode, 0, 0, 0));
  ESP_LOGI(TAG, "I2C0 host initialized");

  /* init TCA6408A group (both @I2C_0) */
  devs_handle_p->tca_dev[0].address = HAND_BUS_I2C_ADDR_TCA_OTHER;
  devs_handle_p->tca_dev[0].i2c_bus = I2C_NUM_0;
  devs_handle_p->tca_dev[1].address = HAND_BUS_I2C_ADDR_TCA_CH101;
  devs_handle_p->tca_dev[1].i2c_bus = I2C_NUM_0;
  ESP_LOGI(TAG, "TCA6408 devices initialized");

  /* TODO: init BQ27427 (if battery is inserted) */
  // TODO: read battery and vbus pin

  /* init VL53L1X group */
  VL53L1_ConfigTca6408a(I2C_NUM_0);

  // TODO: workaround, need refactor
  const uint8_t vl53l1x_1_index = 1;
  const uint8_t vl53l1x_2_index = 2;
  const VL53L1_DEV vl53l1x_1 = &(devs_handle_p->vl53l1x_dev[0]);
  const VL53L1_DEV vl53l1x_2 = &(devs_handle_p->vl53l1x_dev[1]);

  VL53L1_GpioXshutdown(vl53l1x_1_index, HAND_GPIO_LEVEL_LOW);
  VL53L1_GpioXshutdown(vl53l1x_2_index, HAND_GPIO_LEVEL_LOW);

  /* change VL53L1X_1 i2c address to 0x60 */
  VL53L1_GpioXshutdown(vl53l1x_1_index, HAND_GPIO_LEVEL_HIGH);
  VL53L1_WaitDeviceBooted(vl53l1x_1);
  VL53L1X_SetI2CAddress(vl53l1x_1, HAND_BUS_I2C_ADDR_VL53L1X_1);

  /* change VL53L1X_2 i2c address to 0x62 */

  VL53L1_GpioXshutdown(vl53l1x_2_index, HAND_GPIO_LEVEL_HIGH);
  VL53L1_WaitDeviceBooted(vl53l1x_2);
  VL53L1X_SetI2CAddress(vl53l1x_2, HAND_BUS_I2C_ADDR_VL53L1X_2);

  vl53l1x_init(vl53l1x_1, 0);
  vl53l1x_init(vl53l1x_2, 0);

  /* XXX: init I2C1 (I2C for CH101 sensor), take place at ch_init */

  /* init CH101 group */
  ch_group_t* ch101_group_p = &(devs_handle_p->ch101_group);
  ch_dev_t* ch101_devs_p = &(devs_handle_p->ch101_dev);
  chbsp_board_init(ch101_group_p);

  /* show macros defined in app_version.h of lib CH101 */
  ESP_LOGI(TAG, "CHX01 Information");
  ESP_LOGI(TAG, "Compile time:  %s %s", __DATE__, __TIME__);
  ESP_LOGI(TAG, "Version: %u.%u.%u", APP_VERSION_MAJOR, APP_VERSION_MINOR,
           APP_VERSION_REV);
  ESP_LOGI(TAG, "SonicLib version: %u.%u.%u\n", SONICLIB_VER_MAJOR,
           SONICLIB_VER_MINOR, SONICLIB_VER_REV);

  uint8_t num_ports = ch_get_num_ports(ch101_group_p);

  ESP_LOGI(TAG, "Initializing CH101 sensor(s)... ");

  uint8_t chirp_err;

  for (uint8_t dev_num = 0; dev_num < num_ports; dev_num++)
  {
    ch_dev_t* dev_ptr = ch101_devs_p + dev_num;  // init struct in array

    /* Init device descriptor
     *   Note that this assumes all sensors will use the same sensor
     *   firmware.  The CHIRP_SENSOR_FW_INIT_FUNC symbol is defined in
     *   app_config.h and is used for all devices.
     *
     *   However, it is possible for different sensors to use different
     *   firmware images, by specifying different firmware init routines
     *   when ch_init() is called for each.
     */
    chirp_err |=
        ch_init(dev_ptr, ch101_group_p, dev_num, CHIRP_SENSOR_FW_INIT_FUNC);
  }

  if (chirp_err == 0)
  {
    ESP_LOGI(TAG, "Starting CH101 group (burn firmaware...)");
    chirp_err = ch_group_start(ch101_group_p);
  }
  else
  {
    ESP_LOGE(TAG, "CH101 Init failed, {%d}!", chirp_err);
  }

  if (chirp_err != 0)
  {
    ESP_LOGE(TAG, "CH101 State failed, {%d}!", chirp_err);
  }

  ESP_LOGI(TAG, "Sensor\tType \t   Freq\t\t RTC Cal \tFirmware");
  for (uint8_t dev_num = 0; dev_num < num_ports; dev_num++)
  {
    ch_dev_t* dev_ptr = ch_get_dev_ptr(ch101_group_p, dev_num);

    if (ch_sensor_is_connected(dev_ptr))
    {
      ESP_LOGI(
          TAG, "%d\tCH%d\t %u Hz\t%u@%ums\t%s", dev_num,
          ch_get_part_number(dev_ptr), (unsigned int)ch_get_frequency(dev_ptr),
          ch_get_rtc_cal_result(dev_ptr), ch_get_rtc_cal_pulselength(dev_ptr),
          ch_get_fw_version_string(dev_ptr));
    }
  }

  /* Note: to pass compiling */
  ch_thresholds_t chirp_ch201_thresholds = {.threshold = {
                                                {0, 5000},  /* threshold 0 */
                                                {26, 2000}, /* threshold 1 */
                                                {39, 800},  /* threshold 2 */
                                                {56, 400},  /* threshold 3 */
                                                {79, 250},  /* threshold 4 */
                                                {89, 175}   /* threshold 5 */
                                            }};

  /* TODO: should we add callback here?  */

  /* Configure CH101 group */
  uint8_t num_connected_sensors = 0;

  ESP_LOGI(TAG, "Configuring CH101 sensor(s)...");
  for (uint8_t dev_num = 0; dev_num < num_ports; dev_num++)
  {
    ch_config_t dev_config;
    ch_dev_t* dev_ptr = ch_get_dev_ptr(ch101_group_p, dev_num);

    if (ch_sensor_is_connected(dev_ptr))
    {
      /* Select sensor mode
       *   All connected sensors are placed in hardware triggered mode.
       *   The first connected (lowest numbered) sensor will transmit and
       *   receive, all others will only receive.
       */

      num_connected_sensors++;  // count one more connected
      hand_global_ch101_active_dev_num |=
          (1 << dev_num);  // add to active device bit mask

      if (num_connected_sensors == 1)
      {  // if this is the first sensor
        dev_config.mode = CHIRP_FIRST_SENSOR_MODE;
      }
      else
      {
        dev_config.mode = CHIRP_OTHER_SENSOR_MODE;
      }

      if (dev_config.mode != CH_MODE_FREERUN)
      {                                         // unless free-running
        hand_global_ch101_triggered_dev_num++;  // will be triggered
      }

      /* Init config structure with values from app_config.h */
      dev_config.max_range = CHIRP_SENSOR_MAX_RANGE_MM;
      dev_config.static_range = CHIRP_SENSOR_STATIC_RANGE;

      /* If sensor will be free-running, set internal sample interval */
      if (dev_config.mode == CH_MODE_FREERUN)
      {
        dev_config.sample_interval = MEASUREMENT_INTERVAL_MS;
      }
      else
      {
        dev_config.sample_interval = 0;
      }

      /* Set detection thresholds (CH201 only) */
      if (ch_get_part_number(dev_ptr) == CH201_PART_NUMBER)
      {
        /* Set pointer to struct containing detection thresholds */
        ESP_LOGE(
            "app_main",
            "ch-201 threshold setting should not be called in ch-101 test!");
        dev_config.thresh_ptr = &chirp_ch201_thresholds;
      }
      else
      {
        dev_config.thresh_ptr = 0;
      }

      /* Apply sensor configuration */
      chirp_err = ch_set_config(dev_ptr, &dev_config);

      /* Enable sensor interrupt if using free-running mode
       *   Note that interrupt is automatically enabled if using
       *   triggered modes.
       */
      if ((!chirp_err) && (dev_config.mode == CH_MODE_FREERUN))
      {
        chbsp_set_io_dir_in(dev_ptr);
        chbsp_io_interrupt_enable(dev_ptr);
      }

      /* Read back and display config settings */
      if (!chirp_err)
      {
        ultrasound_display_config_info(dev_ptr);
      }
      else
      {
        ESP_LOGI(TAG, "Device %d: Error during ch_set_config()\n", dev_num);
      }

      /* Turn on an LED to indicate device connected */
      if (!chirp_err)
      {
        chbsp_led_on(dev_num);
        ESP_LOGI(TAG, "CH101 sensor(s): {%d} config done!", dev_num);
      }
    }
  }
  /**
   * @brief 目前缺少了
   *
   * @note - callback 註冊 -> app main?
   * @note - ch_set_rx_pretrigger -> need?
   * @note - 啟動 periodic timer -> app_main
   *
   */

  return ret;
}
static esp_err_t hand_spi_bus_and_device_init(
    hand_devices_handle_t* devs_handle_p)
{
  esp_err_t ret = ESP_OK;

  /* init SPI2 */
  spi_host_device_t spi2_host = SPI2_HOST;
  spi_bus_config_t spi2_buscfg = {
      .mosi_io_num = HAND_PIN_SPI_2_MOSI,
      .miso_io_num = HAND_PIN_SPI_2_MISO,
      .sclk_io_num = HAND_PIN_SPI_2_SCLK,
      .quadwp_io_num = HAND_PIN_DUMMY,
      .quadhd_io_num = HAND_PIN_DUMMY,
      .max_transfer_sz = HAND_SIZE_SPI2_TRANSFER,
      .flags = 0,
      .intr_flags = 0,
      // TODO: wait for testing
      // .isr_cpu_id = HAND_CPU_ID_SPI2
  };

  ESP_ERROR_CHECK(spi_bus_initialize(spi2_host, &spi2_buscfg, SPI_DMA_CH_AUTO));
  ESP_LOGI(TAG, "SPI2 host initialized");

  /* TODO: mount BMI323, use library */
  spi_device_interface_config_t bmi323_devcfg = {
      .clock_speed_hz = HAND_BUS_SPI_BMI323_SPEED,
      .mode = HAND_BUS_SPI_BMI323_MODE,
      .spics_io_num = HAND_PIN_CS_BMI323,
      .queue_size = HAND_SIZE_SPI2_BMI323_QUEUE,
      // TODO: create a callback
      // .post_cb = ??
  };

  ESP_ERROR_CHECK(spi_bus_add_device(spi2_host, &bmi323_devcfg,
                                     &(devs_handle_p->bmi323_handle)));
  ESP_LOGI(TAG, "BMI323 SPI initialized (@SPI-{2})");

  /* TODO: BMI323 init */
  ESP_LOGW(TAG, "Not implemented: BMI323 device init");

  /* mount KX132_1211_1 */
  spi_device_interface_config_t kx132_1_devcfg = {
      .clock_speed_hz = HAND_BUS_SPI_KX132_SPEED,
      .mode = HAND_BUS_SPI_KX132_MODE,
      .spics_io_num = HAND_PIN_CS_KX132_1,
      .queue_size = HAND_SIZE_SPI2_KX132_QUEUE,
      // TODO: create a callback
      // .post_cb = ??
  };

  ESP_ERROR_CHECK(spi_bus_add_device(spi2_host, &kx132_1_devcfg,
                                     &(devs_handle_p->kx132_1_dev_handle)));
  ESP_LOGI(TAG, "KX132-1211_1 SPI initialized (@SPI-{2})");

  /* TODO: BMI323 init */
  ESP_LOGW(TAG, "Not implemented: KX132-1211_1 device init");

  /* mount BOS1901 group */
  spi_device_interface_config_t bos1901_1_devcfg = {
      .clock_speed_hz = HAND_BUS_SPI_BOS1901_SPEED,
      .mode = HAND_BUS_SPI_BOS1901_MODE,
      .spics_io_num = HAND_PIN_CS_BOS1901_1,
      .queue_size = HAND_SIZE_SPI2_BOS1901_QUEUE,
      // TODO: create a callback
      // .post_cb = ??
  };

  spi_device_interface_config_t bos1901_2_devcfg = {
      .clock_speed_hz = HAND_BUS_SPI_BOS1901_SPEED,
      .mode = HAND_BUS_SPI_BOS1901_MODE,
      .spics_io_num = HAND_PIN_CS_BOS1901_2,
      .queue_size = HAND_SIZE_SPI2_BOS1901_QUEUE,
      // TODO: create a callback
      // .post_cb = ??
  };

  spi_device_interface_config_t bos1901_3_devcfg = {
      .clock_speed_hz = HAND_BUS_SPI_BOS1901_SPEED,
      .mode = HAND_BUS_SPI_BOS1901_MODE,
      .spics_io_num = HAND_PIN_CS_BOS1901_3,
      .queue_size = HAND_SIZE_SPI2_BOS1901_QUEUE,
      // TODO: create a callback
      // .post_cb = ??
  };

  spi_device_interface_config_t bos1901_4_devcfg = {
      .clock_speed_hz = HAND_BUS_SPI_BOS1901_SPEED,
      .mode = HAND_BUS_SPI_BOS1901_MODE,
      .spics_io_num = HAND_PIN_CS_BOS1901_4,
      .queue_size = HAND_SIZE_SPI2_BOS1901_QUEUE,
      // TODO: create a callback
      // .post_cb = ??
  };

  /* bos1901 SPI device config */
  bos1901_spi_dev_config_t bos1901_1_spi_devcfg = {
      .target_bus = spi2_host,
      .dev_config = bos1901_1_devcfg,
      .handle_ptr = &(devs_handle_p->bos1901_1_spi_handle)};

  bos1901_spi_dev_config_t bos1901_2_spi_devcfg = {
      .target_bus = spi2_host,
      .dev_config = bos1901_2_devcfg,
      .handle_ptr = &(devs_handle_p->bos1901_2_spi_handle)};

  bos1901_spi_dev_config_t bos1901_3_spi_devcfg = {
      .target_bus = spi2_host,
      .dev_config = bos1901_3_devcfg,
      .handle_ptr = &(devs_handle_p->bos1901_3_spi_handle)};

  bos1901_spi_dev_config_t bos1901_4_spi_devcfg = {
      .target_bus = spi2_host,
      .dev_config = bos1901_4_devcfg,
      .handle_ptr = &(devs_handle_p->bos1901_4_spi_handle)};

  /* bos1901 device config */

  // allocate bos1901_dev_handle_t and assign new address to handle
  devs_handle_p->bos1901_1_dev_handle = bos1901_device_create("HAND_BOS1901_1");
  devs_handle_p->bos1901_2_dev_handle = bos1901_device_create("HAND_BOS1901_2");
  devs_handle_p->bos1901_3_dev_handle = bos1901_device_create("HAND_BOS1901_3");
  devs_handle_p->bos1901_4_dev_handle = bos1901_device_create("HAND_BOS1901_4");

  ESP_LOGI(TAG, "BOS1901 group SPI initialized (@SPI-{2})");

  // init bos1901 group
  bos1901_device_init(devs_handle_p->bos1901_1_dev_handle, NULL,
                      &bos1901_1_spi_devcfg);
  bos1901_device_init(devs_handle_p->bos1901_2_dev_handle, NULL,
                      &bos1901_2_spi_devcfg);
  bos1901_device_init(devs_handle_p->bos1901_3_dev_handle, NULL,
                      &bos1901_3_spi_devcfg);
  bos1901_device_init(devs_handle_p->bos1901_4_dev_handle, NULL,
                      &bos1901_4_spi_devcfg);

  ESP_LOGI(TAG, "BOS1901 group device initialized");

  /* init SPI3 */
  spi_host_device_t spi3_host = SPI3_HOST;
  spi_bus_config_t spi3_buscfg = {
      .mosi_io_num = HAND_PIN_SPI_3_MOSI,
      .miso_io_num = HAND_PIN_SPI_3_MISO,
      .sclk_io_num = HAND_PIN_SPI_3_SCLK,
      .quadwp_io_num = HAND_PIN_DUMMY,
      .quadhd_io_num = HAND_PIN_DUMMY,
      .max_transfer_sz = HAND_SIZE_SPI3_TRANSFER,
      .flags = 0,
      .intr_flags = 0,
      // TODO: wait for testing
      // .isr_cpu_id = HAND_CPU_ID_SPI3
  };

  ESP_ERROR_CHECK(spi_bus_initialize(spi3_host, &spi3_buscfg, SPI_DMA_CH_AUTO));
  ESP_LOGI(TAG, "SPI3 host initialized");

  /* mount rest KX132_1211 group */
  spi_device_interface_config_t kx132_2_devcfg = {
      .clock_speed_hz = HAND_BUS_SPI_KX132_SPEED,
      .mode = HAND_BUS_SPI_KX132_MODE,
      .spics_io_num = HAND_PIN_CS_KX132_2,
      .queue_size = HAND_SIZE_SPI3_KX132_QUEUE,
      // TODO: create a callback
      // .post_cb = ??
  };

  spi_device_interface_config_t kx132_3_devcfg = {
      .clock_speed_hz = HAND_BUS_SPI_KX132_SPEED,
      .mode = HAND_BUS_SPI_KX132_MODE,
      .spics_io_num = HAND_PIN_CS_KX132_3,
      .queue_size = HAND_SIZE_SPI3_KX132_QUEUE,
      // TODO: create a callback
      // .post_cb = ??
  };

  spi_device_interface_config_t kx132_4_devcfg = {
      .clock_speed_hz = HAND_BUS_SPI_KX132_SPEED,
      .mode = HAND_BUS_SPI_KX132_MODE,
      .spics_io_num = HAND_PIN_CS_KX132_4,
      .queue_size = HAND_SIZE_SPI3_KX132_QUEUE,
      // TODO: create a callback
      // .post_cb = ??
  };

  ESP_ERROR_CHECK(spi_bus_add_device(spi3_host, &kx132_2_devcfg,
                                     &(devs_handle_p->kx132_2_dev_handle)));
  ESP_LOGI(TAG, "KX132-1211_2 SPI initialized (@SPI-{3})");
  ESP_ERROR_CHECK(spi_bus_add_device(spi3_host, &kx132_3_devcfg,
                                     &(devs_handle_p->kx132_3_dev_handle)));
  ESP_LOGI(TAG, "KX132-1211_3 SPI initialized (@SPI-{3})");
  ESP_ERROR_CHECK(spi_bus_add_device(spi3_host, &kx132_4_devcfg,
                                     &(devs_handle_p->kx132_4_dev_handle)));
  ESP_LOGI(TAG, "KX132-1211_4 SPI initialized (@SPI-{3})");

  return ret;
}

/* GPIO related */

static esp_err_t hand_gpio_init_vl53l1x()
{
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.pin_bit_mask = (1ULL << HAND_PIN_INT_VL53L1X_1_GPIO1) |
                         (1ULL << HAND_PIN_INT_VL53L1X_2_GPIO1);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  return gpio_config(&io_conf);
}

static esp_err_t hand_gpio_init(hand_devices_handle_t* devs_handle_p)
{
  esp_err_t ret = ESP_OK;

  /* VL53L1X related */
  hand_gpio_init_vl53l1x();

  /* LED related (RMT) */
  led_strip_config_t strip_config = {
      .strip_gpio_num = HAND_PIN_DEBUG_RGB_LED,  // The GPIO that connected to
                                                 // the LED strip's data line
      .max_leds = 1,  // The number of LEDs in the strip,
      .led_pixel_format =
          LED_PIXEL_FORMAT_GRB,       // Pixel format of your LED strip
      .led_model = LED_MODEL_WS2812,  // LED strip model
      .flags.invert_out = false,  // whether to invert the output signal (useful
                                  // when your hardware has a level inverter)
  };

  led_strip_rmt_config_t rmt_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT,     // different clock source can lead to
                                          // different power consumption
      .resolution_hz = 10 * 1000 * 1000,  // 10MHz
      .flags.with_dma = false,            // whether to enable the DMA feature
  };

  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config,
                                           &(devs_handle_p->rgb_led_handle)));

  /* TODO: Activate LED toggle */
}

static esp_err_t hand_isr_init()
{
  esp_err_t ret = ESP_OK;

  /* TODO: check this could be deleted or not? */
  /* TODO: if confirm can be remove, remove the one in CH101 */
  // No INTR flag, should failed (ch101 lib already use it)
  gpio_install_isr_service(0);

  /* add vl53l1x related isr callback  */
  gpio_isr_handler_add(HAND_PIN_INT_VL53L1X_1_GPIO1, hand_cb_vl53l1x_sensed,
                       (void*)HAND_PIN_INT_VL53L1X_1_GPIO1);
  gpio_isr_handler_add(HAND_PIN_INT_VL53L1X_2_GPIO1, hand_cb_vl53l1x_sensed,
                       (void*)HAND_PIN_INT_VL53L1X_2_GPIO1);
}

static esp_err_t hand_data_server_init() { esp_err_t ret = ESP_OK; }

esp_err_t hand_init(const char* ssid, const char* password, bool init_dev)
{
  esp_err_t ret = ESP_OK;

  /* wait for system starts if usb plugged in */
  if (usb_serial_jtag_is_connected())
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  ret = hand_wifi_module_mount(NULL);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "hand_wifi_module_mount failed");
    return ret;
  }

  hand_wifi_t wifi_settings;

  ret = hand_wifi_module_set_handle(&wifi_settings, ssid, password);

  // OR

  // ret = hand_wifi_module_get_default_handle(&wifi_settings);

  /* Note: modify the ssid and password according to your config */
  // const char* new_ssid = "NEW_SSID";
  // memcpy(&wifi_settings.config.ssid, &new_ssid, sizeof(new_ssid));
  // const char* new_password = "NEW_PASSWORD";
  // memcpy(&wifi_settings.config.password, &new_password,
  // sizeof(new_password));

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "hand_wifi_module_get_default_handle failed");
    return ret;
  }

  ESP_LOGI(TAG, "Default Wi-Fi SSID: %s", (char*)&wifi_settings.config.ssid);
  ESP_LOGI(TAG, "Default Wi-Fi Password: %s",
           (char*)&wifi_settings.config.password);

  hand_wifi_module_update_handler(NULL);

  /* should connect */
  ret = hand_wifi_module_init(&wifi_settings, true);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "hand_wifi_module_init failed");
    return ret;
  }

  hand_terminal_t terminal_setting = {
      .local_server = {.server_type = HAND_UDP_SERVER,
                       .fcntl_flag = O_NONBLOCK,
                       .addr_family = HAND_AF_INET,
                       .addr = {.port = 12345}},
      .dest_addr = {.ip = HAND_DEFAULT_LOG_SERVER_IP, .port = 12345}};

  ret = hand_terminal_module_mount(NULL);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "hand_terminal_module_mount failed");
    return ret;
  }

  ret = hand_terminal_module_init(&terminal_setting);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "hand_terminal_module_init failed");
    return ret;
  }

  /* Init hand hardware begin */
  if (init_dev)
  {
    hand_spi_bus_and_device_init(&hand_global_devs_handle);
    hand_i2c_bus_and_device_init(&hand_global_devs_handle);
    hand_gpio_init(&hand_global_devs_handle);
    hand_isr_init();
    /* TODO: get all gpio states */
    // gpio_dump_all_io_configuration();
  }

  /* init global variable */
  ret = hand_global_var_init();

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Init global variable failed!");
  }

  return ret;
}