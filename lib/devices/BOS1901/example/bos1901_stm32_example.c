// main.c (STM32 example)

/* FIXME */
#include "bos1901.h"
#include "bos1901_stm32_hal.h"

void setup_spi() {
    // initialize the SPI handle (hspi1 should be configured in STM32CubeMX)
    extern SPI_HandleTypeDef hspi1;

    // configure the SPI device
    bos1901_spi_dev_config_t stm32_hal_device_config = {
        .handle = &hspi1,
        .cs_pin = {
            .gpio_x = GPIOB, // example GPIO port
            .gpio_pin = GPIO_PIN_12, // example GPIO pin
        }
    };

    // initialize the SPI device
    bos1901_spi_dev_t spi_dev;
    spi_dev.spi_interface_ptr = &bos1901_stm32_hal_spi;
    bos1901_spi_device_init(&spi_dev, &stm32_hal_device_config);

    // write data
    uint8_t data_to_write = 0x55;
    bos1901_spi_device_write(&spi_dev, &data_to_write, 1);

    // read data
    uint8_t data_to_read;
    bos1901_spi_device_read(&spi_dev, &data_to_read, 1);

    // de-initialize the SPI device
    bos1901_change_spi_interface(&spi_dev, &bos1901_dummy_spi);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    // setup peripherals
    setup_spi();

    while (1) {
        // main loop
    }
}
