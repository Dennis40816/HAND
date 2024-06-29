#include "hand_common.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "bos1901.h"

// SPI config
#define PIN_NUM_MISO  11
#define PIN_NUM_MOSI  13
#define PIN_NUM_CLK   12
#define PIN_NUM_CS0   10
#define PIN_NUM_CS1   9
#define PIN_NUM_CS2   8
#define PIN_NUM_CS3   7
#define PIN_KX_CS     6
#define PIN_BMI_CS    5
#define PIN_LED_RED   43
#define PIN_LED_WHITE 44
#define HSPI_HOST     (SPI2_HOST)

static const char *TAG = "HAND_BOS1901_MOTOR_TEST";

/* User must declare this */
spi_device_handle_t spi_handle_0;
spi_device_handle_t spi_handle_1;
spi_device_handle_t spi_handle_2;
spi_device_handle_t spi_handle_3;

bos1901_dev_t *bos1901_device_0;
bos1901_dev_t *bos1901_device_1;
bos1901_dev_t *bos1901_device_2;
bos1901_dev_t *bos1901_device_3;

static void init_spi()
{
  spi_bus_config_t buscfg = {
      .miso_io_num = PIN_NUM_MISO,
      .mosi_io_num = PIN_NUM_MOSI,
      .sclk_io_num = PIN_NUM_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 4096,
  };

  esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_LOGI(TAG, "spi bus initialize ret value is: %d. should be 0", ret);

  spi_device_interface_config_t devcfg0 = {
      .clock_speed_hz = 35 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_NUM_CS0,
      .queue_size = 5,
  };

  spi_device_interface_config_t devcfg1 = {
      .clock_speed_hz = 35 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_NUM_CS1,
      .queue_size = 5,
  };

  spi_device_interface_config_t devcfg2 = {
      .clock_speed_hz = 35 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_NUM_CS2,
      .queue_size = 5,
  };

  spi_device_interface_config_t devcfg3 = {
      .clock_speed_hz = 35 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_NUM_CS3,
      .queue_size = 5,
  };

  bos1901_spi_dev_config_t espidf_device_config_0 = {
      .target_bus = SPI2_HOST,
      .handle_ptr = &spi_handle_0,
      .dev_config = devcfg0,
  };

  bos1901_spi_dev_config_t espidf_device_config_1 = {
      .target_bus = SPI2_HOST,
      .handle_ptr = &spi_handle_1,
      .dev_config = devcfg1,
  };

  bos1901_spi_dev_config_t espidf_device_config_2 = {
      .target_bus = SPI2_HOST,
      .handle_ptr = &spi_handle_2,
      .dev_config = devcfg2,
  };

  bos1901_spi_dev_config_t espidf_device_config_3 = {
      .target_bus = SPI2_HOST,
      .handle_ptr = &spi_handle_3,
      .dev_config = devcfg3,
  };

  bos1901_device_0 = bos1901_device_create("BOS1901_Device_0");
  bos1901_device_1 = bos1901_device_create("BOS1901_Device_1");
  bos1901_device_2 = bos1901_device_create("BOS1901_Device_2");
  bos1901_device_3 = bos1901_device_create("BOS1901_Device_3");

  bos1901_device_init(bos1901_device_0, NULL, &espidf_device_config_0);
  bos1901_device_init(bos1901_device_1, NULL, &espidf_device_config_1);
  bos1901_device_init(bos1901_device_2, NULL, &espidf_device_config_2);
  bos1901_device_init(bos1901_device_3, NULL, &espidf_device_config_3);
}

static int16_t volt_2_amp(float volt)
{
  int16_t amplitude = volt * 2047 / 3.6 / 31;

  return amplitude & 0x0FFF;
}

static void led_init()
{
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pin_bit_mask = (1ULL << PIN_LED_RED) | (1ULL << PIN_LED_WHITE);
  gpio_config(&io_conf);

  gpio_set_level(PIN_LED_RED, 0);
  gpio_set_level(PIN_LED_WHITE, 0);
}

static void led_off() { gpio_set_level(PIN_LED_RED, 0); }

static void led_on() { gpio_set_level(PIN_LED_RED, 1); }

static void set_other_cs_pins_high()
{
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pin_bit_mask = ((1ULL << PIN_KX_CS) | (1ULL << PIN_BMI_CS));

  gpio_config(&io_conf);

  gpio_set_level(PIN_KX_CS, 1);
  gpio_set_level(PIN_BMI_CS, 1);

  ESP_LOGI("GPIO", "GPIO %d (KX132-1211 CS) set to output high", PIN_KX_CS);
  ESP_LOGI("GPIO", "GPIO %d (BMI323 CS) set to output high", PIN_BMI_CS);
}

static void test_bos1901_device()
{
  // Set unused CS pins high
  set_other_cs_pins_high();
  led_init();

  // Send reset command
  bos1901_device_reset(bos1901_device_0);
  bos1901_device_reset(bos1901_device_1);
  bos1901_device_reset(bos1901_device_2);
  bos1901_device_reset(bos1901_device_3);

  // Wait 50 ms
  vTaskDelay(pdMS_TO_TICKS(50));

  // Read
  uint16_t expected_value = 0x246A;

  uint16_t read_value_0;
  uint16_t read_value_1;
  uint16_t read_value_2;
  uint16_t read_value_3;

  bos1901_device_read_reg(bos1901_device_0, BOS1901_REG_ID, &read_value_0);
  bos1901_device_read_reg(bos1901_device_1, BOS1901_REG_ID, &read_value_1);
  bos1901_device_read_reg(bos1901_device_2, BOS1901_REG_ID, &read_value_2);
  bos1901_device_read_reg(bos1901_device_3, BOS1901_REG_ID, &read_value_3);

  // Assert register value is 0x246A
  ESP_LOGI(TAG, "Read_value_0: 0x%04X, expected: 0x%04X", read_value_0,
           expected_value);
  ESP_LOGI(TAG, "Read_value_1: 0x%04X, expected: 0x%04X", read_value_1,
           expected_value);
  ESP_LOGI(TAG, "Read_value_2: 0x%04X, expected: 0x%04X", read_value_2,
           expected_value);
  ESP_LOGI(TAG, "Read_value_3: 0x%04X, expected: 0x%04X\n\n", read_value_3,
           expected_value);
}

void app_main()
{
  // XXX: set * to ESPLOG_VERBOSE cause unknown problem
  esp_log_level_set("bos1901", ESP_LOG_VERBOSE);

  // Wait for devices to stabilize
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Initialize SPI bus and devices
  init_spi();

  // Run the test
  test_bos1901_device();

  /* need to change bos1901 play, upi, sq, lmi reg  */

  // for sq
  // Enter a hexadecimal value: 0x0080
  // Hex: 0x0080 -> Formatted Binary: 0b 0000 0000 1000 0000

  // Field Names: KP
  // 0b 0111 0011 1010 (0x0080)
  //    |-------------
  //    |      |_______KP
  //    |_______________SQ
  uint16_t kp_reg_val;
  bos1901_device_read_reg(bos1901_device_0, BOS1901_REG_KP, &kp_reg_val);
  // remove MSB byte (indicate read reg address)
  kp_reg_val &= 0xFFF;
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "Got KP     REG value: 0x%04X. EXPECTED VALUE is {0x0080}",
           kp_reg_val);

  // for lmi, upi
  // Enter a hexadecimal value: 0x073A
  // Hex: 0x073A -> Formatted Binary: 0b 0000 0111 0011 1010

  // Field Names: PARCAP
  // 0b 0111 0011 1010 (0x073A)
  //    |||| ---------
  //    ||||     |_______PARCAP [7:0]
  //    ||||______________CAL
  //    |||________________CP5
  //    ||_________________ LMI
  //    |___________________ UPI
  uint16_t parcap_reg_val;
  bos1901_device_read_reg(bos1901_device_0, BOS1901_REG_PARCAP,
                          &parcap_reg_val);
  // remove MSB byte (indicate read reg address)
  parcap_reg_val &= 0xFFF;
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "Got PARCAP REG value: 0x%04X. EXPECTED VALUE is {0x073A}",
           parcap_reg_val);

  // for play mode CONFIG
  // Enter a hexadecimal value: 0x0280
  // Hex: 0x0280 -> Formatted Binary: 0b 0000 0010 1000 0000 (0x0280)
  //                                          ------ BC (0b101 is CONFIG)
  uint16_t config_reg_val;
  bos1901_device_read_reg(bos1901_device_0, BOS1901_REG_CONFIG,
                          &config_reg_val);
  // remove MSB byte (indicate read reg address)
  config_reg_val &= 0xFFF;
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "Got CONFIG REG value: 0x%04X. EXPECTED VALUE is {0x0280}",
           config_reg_val);

  // modify KP config
  const uint16_t SQ_FIELD_VAL = BOS1901_SQ_SQUARE_WAVE;
  const uint16_t SQ_FIELD_SHIFT = 11;
  const uint16_t SQ_FIELD_MASK_VALUE = 0b1;
  const uint16_t SQ_FIELD_MASK = SQ_FIELD_MASK_VALUE << SQ_FIELD_SHIFT;

  const uint16_t new_kp_value =
      (kp_reg_val & (~SQ_FIELD_MASK)) | (SQ_FIELD_VAL << SQ_FIELD_SHIFT);

  bos1901_device_write_reg(bos1901_device_0, BOS1901_REG_KP, new_kp_value);

  // modify PARCAP
  const uint16_t UPI_FIELD_VAL = BOS1901_UPI_ENABLE;
  const uint16_t UPI_FIELD_SHIFT = 11;
  const uint16_t UPI_FIELD_MASK_VALUE = 0b1;
  const uint16_t UPI_FIELD_MASK = UPI_FIELD_MASK_VALUE << UPI_FIELD_SHIFT;

  const uint16_t LMI_FIELD_VAL = BOS1901_LMI_OVERRIDE;
  const uint16_t LMI_FIELD_SHIFT = 10;
  const uint16_t LMI_FIELD_MASK_VALUE = 0b1;
  const uint16_t LMI_FIELD_MASK = LMI_FIELD_MASK_VALUE << LMI_FIELD_SHIFT;

  const uint16_t PARCAP_MASK = ~(UPI_FIELD_MASK | LMI_FIELD_MASK);

  const uint16_t new_parcap_value = (parcap_reg_val & PARCAP_MASK) |
                                    (UPI_FIELD_VAL << UPI_FIELD_SHIFT) |
                                    (LMI_FIELD_VAL << LMI_FIELD_SHIFT);

  bos1901_device_write_reg(bos1901_device_0, BOS1901_REG_PARCAP,
                           new_parcap_value);

  // modify config (do not update CONFIG directly!!!)
  const uint16_t PLAY_FIELD_VAL = BOS1901_PLAY_8_KSPS;  // play
  const uint16_t PLAY_FIELD_SHIFT = 0;
  const uint16_t PLAY_FIELD_MASK_VALUE = 0b111;
  const uint16_t PLAY_FIELD_MASK = PLAY_FIELD_MASK_VALUE << PLAY_FIELD_SHIFT;

  const uint16_t new_config_value = (config_reg_val & (~PLAY_FIELD_MASK)) |
                                    (PLAY_FIELD_VAL << PLAY_FIELD_SHIFT);

  bos1901_device_write_reg(bos1901_device_0, BOS1901_REG_CONFIG,
                           new_config_value);

  /* NOTE: do not forget to update dev_config, this will effect read reg
   * operation of CONFIG */
  bos1901_device_0->dev_config->play_mode = BOS1901_PLAY_8_KSPS;

  /* check value (please don't change the order, or config reg check will
   * fail)*/
  uint16_t config_reg_new_val;
  uint16_t kp_reg_new_val;
  uint16_t parcap_reg_new_val;

  bos1901_device_read_reg(bos1901_device_0, BOS1901_REG_PARCAP,
                          &parcap_reg_new_val);
  bos1901_device_read_reg(bos1901_device_0, BOS1901_REG_KP, &kp_reg_new_val);
  bos1901_device_read_reg(bos1901_device_0, BOS1901_REG_CONFIG,
                          &config_reg_new_val);

  parcap_reg_new_val &= 0xFFF;
  kp_reg_new_val &= 0xFFF;
  config_reg_new_val &= 0xFFF;

  ESP_LOGI(TAG, "After modification, KP result: {0x%04X}, should be: {0x%04X}",
           kp_reg_new_val, new_kp_value);
  ESP_LOGI(TAG,
           "After modification, PARCAP result: {0x%04X}, should be: {0x%04X}",
           parcap_reg_new_val, new_parcap_value);
  ESP_LOGI(TAG,
           "After modification, CONFIG result: {0x%04X}, should be: {0x%04X}",
           config_reg_new_val, new_config_value);

  /* write a value to FIFO */
  ESP_LOGW(TAG, "Write to FIFO with value");
  const float test_voltage = 20.0f;
  const int16_t test_voltage_in_2_com = volt_2_amp(test_voltage);
  bos1901_device_write_reg(bos1901_device_0, BOS1901_REG_FIFO,
                           test_voltage_in_2_com);

  /* enable device */
  bos1901_device_output_enable(bos1901_device_0, BOS1901_OE_ENABLE);
  ESP_LOGW(TAG,
           "Output enabled! The output value should be: {%.3f} Volt. Start "
           "your mesurement. The enable time keep for 10 seconds",
           test_voltage);
  led_on();

  vTaskDelay(pdMS_TO_TICKS(10000));

  /* disable device */
  bos1901_device_output_enable(bos1901_device_0, BOS1901_OE_DISABLE);
  led_off();
  ESP_LOGW(TAG, "Output disabled!");

  /* deinit devices */
  bos1901_device_deinit(bos1901_device_0);
  bos1901_device_deinit(bos1901_device_1);
  bos1901_device_deinit(bos1901_device_2);
  bos1901_device_deinit(bos1901_device_3);
}