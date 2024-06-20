#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "unity.h"
#include "bos1901.h"

// 定义 SPI 引脚
#define PIN_NUM_MISO 11
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK  12
#define PIN_NUM_CS0  10
#define PIN_NUM_CS1  9
#define PIN_NUM_CS2  8
#define PIN_NUM_CS3  7
#define PIN_KX_CS    6
#define PIN_BMI_CS   5
#define HSPI_HOST    (SPI2_HOST)

static const char *TAG = "HAND_BOS1901_TEST";

/* User must declare this */
spi_device_handle_t spi_handle_0;
spi_device_handle_t spi_handle_1;
spi_device_handle_t spi_handle_2;
spi_device_handle_t spi_handle_3;

bos1901_dev_t *bos1901_device_0;
bos1901_dev_t *bos1901_device_1;
bos1901_dev_t *bos1901_device_2;
bos1901_dev_t *bos1901_device_3;

void init_spi()
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
      .clock_speed_hz = 10 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_NUM_CS0,
      .queue_size = 5,
  };

  spi_device_interface_config_t devcfg1 = {
      .clock_speed_hz = 10 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_NUM_CS1,
      .queue_size = 5,
  };

  spi_device_interface_config_t devcfg2 = {
      .clock_speed_hz = 10 * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_NUM_CS2,
      .queue_size = 5,
  };

  spi_device_interface_config_t devcfg3 = {
      .clock_speed_hz = 10 * 1000 * 1000,
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

void set_other_cs_pins_high()
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

void test_bos1901_device()
{
  // Set unused CS pins high
  set_other_cs_pins_high();

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
  bos1901_device_read_reg(bos1901_device_3, BOS1901_REG_IC_STATUS, &read_value_3);

  // Assert register value is 0x246A
  ESP_LOGI(TAG, "Read_value_0: 0x%04X, expected: 0x%04X", read_value_0,
           expected_value);
  ESP_LOGI(TAG, "Read_value_1: 0x%04X, expected: 0x%04X", read_value_1,
           expected_value);
  ESP_LOGI(TAG, "Read_value_2: 0x%04X, expected: 0x%04X", read_value_2,
           expected_value);
  ESP_LOGI(TAG, "Read_value_3: 0x%04X, expected: 0x%04X", read_value_3,
           expected_value);

  TEST_ASSERT_EQUAL_HEX16(expected_value, read_value_0);
  TEST_ASSERT_EQUAL_HEX16(expected_value, read_value_1);
  TEST_ASSERT_EQUAL_HEX16(expected_value, read_value_2);
  TEST_ASSERT_EQUAL_HEX16(expected_value, read_value_3);
}

void app_main(void)
{
  // Initialize Unity
  UNITY_BEGIN();

  // Wait for devices to stabilize
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Initialize SPI bus and devices
  init_spi();

  // Run the test
  RUN_TEST(test_bos1901_device);

  // Clean up resources
  bos1901_device_deinit(bos1901_device_0);
  bos1901_device_deinit(bos1901_device_1);
  bos1901_device_deinit(bos1901_device_2);
  bos1901_device_deinit(bos1901_device_3);

  spi_bus_free(HSPI_HOST);

  // End Unity
  UNITY_END();
}
