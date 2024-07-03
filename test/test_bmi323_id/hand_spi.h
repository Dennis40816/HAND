#ifndef HAND_SPI_H
#define HAND_SPI_H

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define SPI_CLOCK_SPEED_HZ 10000000  // 10 MHz
#define BMI323_DUMMY_WORD  0x0000
#define BMI323_SPI_READ    0x80
#define BMI323_REG_CHIP_ID 0x00

void init_spi_bus(spi_host_device_t host, int mosi_pin, int miso_pin,
                  int sclk_pin);
void disable_other_device();
void init_spi_device(spi_host_device_t host, int cs_pin,
                     spi_device_handle_t* handle);
esp_err_t spi_read_word(spi_device_handle_t handle, uint8_t reg,
                        uint16_t* data);

#endif  // HAND_SPI_H
