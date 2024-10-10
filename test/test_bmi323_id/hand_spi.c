#include "hand_spi.h"

static const char* TAG = "HAND_BMI323";

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

void disable_other_device()
{
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask =
      (1ULL << 7) | (1ULL << 8) | (1ULL << 9) | (1ULL << 10) | (1ULL << 6);
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

  gpio_config(&io_conf);

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
      .mode = 0,
      .spics_io_num = cs_pin,
      .queue_size = 7,
  };

  ESP_ERROR_CHECK(spi_bus_add_device(host, &devcfg, handle));
  ESP_LOGI(TAG, "SPI device added to host %d with CS pin %d", host, cs_pin);
}

esp_err_t spi_read_word(spi_device_handle_t handle, uint8_t reg, uint16_t* data)
{
  spi_transaction_t t = {0};
  uint16_t first_word = ((uint16_t)(reg | BMI323_SPI_READ) << 8);

  uint16_t big_endian_cmd[2] = {__builtin_bswap16(first_word),
                                __builtin_bswap16(BMI323_DUMMY_WORD)};
  uint16_t rx_buf[2] = {0};

  ESP_LOGI(TAG, "Send 0x%04X to device", big_endian_cmd[0]);

  t.length = 32;
  t.tx_buffer = big_endian_cmd;
  t.rx_buffer = rx_buf;

  esp_err_t ret = spi_device_transmit(handle, &t);
  *data = rx_buf[1];

  ESP_LOGI(TAG, "Status: {%d}. Read 0x%04X from register 0x%02X", ret, *data,
           reg);
  return ret;
}
