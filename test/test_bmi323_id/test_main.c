#include <unity.h>
#include "hand_spi.h"

#define SPI2_MOSI_PIN GPIO_NUM_13
#define SPI2_MISO_PIN GPIO_NUM_11
#define SPI2_SCLK_PIN GPIO_NUM_12
#define SPI2_CS1_PIN  GPIO_NUM_5

spi_device_handle_t bmi323;

void setUp(void)
{
  disable_other_device();
  init_spi_bus(SPI2_HOST, SPI2_MOSI_PIN, SPI2_MISO_PIN, SPI2_SCLK_PIN);
  init_spi_device(SPI2_HOST, SPI2_CS1_PIN, &bmi323);
}

void tearDown(void)
{
  spi_bus_remove_device(bmi323);
  spi_bus_free(SPI2_HOST);
}

void test_read_bmi323_chip_id(void)
{
  uint16_t id;
  esp_err_t ret = spi_read_word(bmi323, BMI323_REG_CHIP_ID, &id);
  TEST_ASSERT_EQUAL(ESP_OK, ret);
  TEST_ASSERT_EQUAL_HEX16(0x0043, (id & 0x00FF));
}

void app_main(void)
{
  UNITY_BEGIN();
  RUN_TEST(test_read_bmi323_chip_id);
  UNITY_END();
}
