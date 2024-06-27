#include "hand_common.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

/**
 * @pin config
 * SPI2:
 *   SDI (MOSI): IO 13
 *   SDO (MISO): IO 11
 *   SCLK : IO 12
 *   CS1: IO 6
 *
 * SPI3:
 *   SDI (MOSI): IO 42
 *   SDO (MISO): IO 40
 *   SCLK : IO 41
 *   CS2: IO 39
 *   CS3: IO 37
 *   CS4: IO 35
 *
 * @result should works
 */

#define SPI_CLOCK_SPEED_HZ 10000000  // 10 MHz

#define BMI323_DUMMY_WORD  0x0000
#define BMI323_SPI_READ    0x80
#define BMI323_REG_CHIP_ID 0x00

// SPI2 Pins
#define SPI2_MOSI_PIN GPIO_NUM_13
#define SPI2_MISO_PIN GPIO_NUM_11
#define SPI2_SCLK_PIN GPIO_NUM_12
#define SPI2_CS1_PIN  GPIO_NUM_5

static const char* TAG = "HAND_BMI323_ID_TEST";

void init_spi_bus(spi_host_device_t host, int mosi_pin, int miso_pin,
                  int sclk_pin)
{
  spi_bus_config_t buscfg = {.mosi_io_num = mosi_pin,
                             .miso_io_num = miso_pin,
                             .sclk_io_num = sclk_pin,
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1,
                             .max_transfer_sz = 4096};

  ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));
  ESP_LOGI(TAG, "SPI bus initialized for host %d", host);
}

/* disable all BOS1901 CS pin */
void disable_other_device()
{
  gpio_config_t io_conf;

  // 配置引腳 7, 8, 9, 10 為輸出模式
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask =
      (1ULL << 7) | (1ULL << 8) | (1ULL << 9) | (1ULL << 10) | (1ULL << 6);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

  // 配置 GPIO
  gpio_config(&io_conf);

  // 設置引腳 6, 7, 8, 9, 10 為高電位
  gpio_set_level(6, 1);
  gpio_set_level(7, 1);
  gpio_set_level(8, 1);
  gpio_set_level(9, 1);
  gpio_set_level(10, 1);

  ESP_LOGI(TAG, "Set GPIO 6, 7, 8, 9, 10 to high level");
}

void init_spi_device(spi_host_device_t host, int cs_pin,
                     spi_device_handle_t* handle)
{
  spi_device_interface_config_t devcfg = {
      .clock_speed_hz = SPI_CLOCK_SPEED_HZ,
      .mode = 0,  // SPI mode 0
      .spics_io_num = cs_pin,
      .queue_size = 7,
  };

  ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg, handle));
  ESP_LOGI(TAG, "SPI device added to host %d with CS pin %d", host, cs_pin);
}

esp_err_t spi_read_word(spi_device_handle_t handle, uint8_t reg, uint16_t* data)
{
  spi_transaction_t t = {0};
  uint16_t first_word = ((uint16_t)(reg | BMI323_SPI_READ)
                         << 8);  // low 8 bytes all zero for dummy bytes

  // also you can use [reg | BMI323_SPI_READ, 0x00] to replace first word
  uint16_t big_endian_cmd[2] = {__builtin_bswap16(first_word),
                                __builtin_bswap16(BMI323_DUMMY_WORD)};

  uint16_t rx_buf[2] = {0};

  ESP_LOGI(TAG, "Send 0x%04X to device", big_endian_cmd[0]);

  t.length = 32;
  t.tx_buffer = big_endian_cmd;
  t.rx_buffer = rx_buf;

  esp_err_t ret = spi_device_transmit(handle, &t);

  *data = (rx_buf[1]);

  ESP_LOGI(TAG, "Status: {%d}. Read 0x%04X from register 0x%02X", ret, *data,
           reg);
  return ret;
}

void app_main(void)
{
  vTaskDelay(pdMS_TO_TICKS(2000));

  disable_other_device();

  // Initialize SPI2 bus and device
  init_spi_bus(SPI2_HOST, SPI2_MOSI_PIN, SPI2_MISO_PIN, SPI2_SCLK_PIN);

  spi_device_handle_t bmi323;
  init_spi_device(SPI2_HOST, SPI2_CS1_PIN, &bmi323);

  ESP_LOGI(TAG, "SPI initialization complete");

  ESP_LOGI(TAG, "Try to read BMI323 CHIP ID");
  uint16_t ID;
  spi_read_word(bmi323, BMI323_REG_CHIP_ID, &ID);

  /* other test (should always returns zero) */
  spi_read_word(bmi323, 0x01, &ID);
  spi_read_word(bmi323, 0x7E, &ID);
}