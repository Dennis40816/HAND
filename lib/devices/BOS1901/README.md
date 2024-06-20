# BOS1901 Library

The `bos1901` library provides an interface for the BOS1901 device, supporting multiple platforms with an abstraction layer for SPI communication.

## Table of Contents
- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)
- [Struct Description](#struct-description)
- [Platform Dependent](#platform-dependent)

## Introduction

This library allows for easy integration and communication with the BOS1901 device over SPI. It supports multiple platforms, including ESP-IDF and STM32 HAL, with a dummy interface for testing and documentation purposes.

## Installation

To install the library, simply include the header files in your project and link against the appropriate platform-specific implementations.

## Usage

Here is an example of how to use the library with ESP-IDF and STM32 platforms:

## How should we read data from BOS1901



### ESP-IDF Example

```c
#include "bos1901.h"
#include "bos1901_espidf.h"

void app_main(void) {
    spi_bus_config_t bus_config = {
        .miso_io_num = 25,
        .mosi_io_num = 23,
        .sclk_io_num = 19,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_bus_initialize(HSPI_HOST, &bus_config, 1);

    bos1901_spi_dev_config_t espidf_device_config = {
        .handle = NULL,
        .dev_config = {
            .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
            .mode = 0,
            .spics_io_num = 22,
            .queue_size = 1,
        }
    };

    bos1901_spi_dev_t spi_dev;
    spi_dev.spi_interface_ptr = &bos1901_espidf_spi;
    bos1901_spi_device_init(&spi_dev, &espidf_device_config);

    uint8_t data_to_write = 0xAA;
    bos1901_spi_device_write(&spi_dev, &data_to_write, 1);

    uint8_t data_to_read;
    bos1901_spi_device_read(&spi_dev, &data_to_read, 1);

    bos1901_change_spi_interface(&spi_dev, &bos1901_dummy_spi);
}

### STM32 Example

```c
#include "bos1901.h"
#include "bos1901_stm32_hal.h"

void setup_spi() {
    extern SPI_HandleTypeDef hspi1;

    bos1901_spi_dev_config_t stm32_hal_device_config = {
        .handle = &hspi1,
        .cs_pin = {
            .gpio_x = GPIOB,
            .gpio_pin = GPIO_PIN_12,
        }
    };

    bos1901_spi_dev_t spi_dev;
    spi_dev.spi_interface_ptr = &bos1901_stm32_hal_spi;
    bos1901_spi_device_init(&spi_dev, &stm32_hal_device_config);

    uint8_t data_to_write = 0x55;
    bos1901_spi_device_write(&spi_dev, &data_to_write, 1);

    uint8_t data_to_read;
    bos1901_spi_device_read(&spi_dev, &data_to_read, 1);

    bos1901_change_spi_interface(&spi_dev, &bos1901_dummy_spi);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    setup_spi();

    while (1) {
        // main loop
    }
}
```

## Struct Description (platform independent)

### bos1901_dev_config_t
- **Description**: This is a platform-independent structure that contains configuration settings related to registers and other parameters specific to the BOS1901 device. It is intended to encapsulate all the non-platform-specific configurations for the device.
- **Usage**: Used to configure the BOS1901 device across different platforms, independent of the underlying hardware.

### bos1901_spi_dev_config_t
- **Description**: This structure is used to configure platform-specific settings for the SPI device. It includes platform-dependent details such as SPI handle and chip select pin information.
- **Usage**: This is a generic placeholder that gets typedef’d into specific platform-dependent configurations.

### bos1901_spi_interface_t
- **Description**: This structure defines a set of function pointers for SPI operations. It provides an abstraction for initializing, de-initializing, writing, and reading from the SPI interface.
- **Usage**: This structure is used to encapsulate SPI operations, allowing different implementations for different platforms.

### bos1901_spi_dev_t
- **Description**: This structure holds a reference to the SPI interface and its configuration. It is used to manage the SPI communication for the BOS1901 device.
- **Usage**: This is used to interact with the SPI interface, providing a unified way to manage the SPI communication for the BOS1901 device.

### bos1901_dev_t
- **Description**: This structure is the main configuration structure for the BOS1901 device. It holds a reference to both the platform-independent device configuration (`bos1901_dev_config_t`) and the SPI device configuration (`bos1901_spi_dev_t`).
- **Usage**: This is the primary structure used to manage the BOS1901 device, combining both device-specific and SPI-specific configurations.

## Struct Description (platform dependent)

### bos1901_espidf_spi_dev_config_t
- **Description**: This structure contains the ESP-IDF specific SPI configuration, including the SPI handle and device configuration. It is typedef’d to `bos1901_spi_dev_config_t`.
- **Usage**: Used only when compiling for ESP-IDF, providing the necessary details for SPI communication on ESP32 devices.

### bos1901_stm32_hal_spi_dev_config_t
- **Description**: This structure holds the STM32 HAL specific SPI configuration. It includes the SPI handle and chip select pin configuration. It is typedef’d to `bos1901_spi_dev_config_t`.
- **Usage**: Used only when compiling for STM32 HAL, providing the necessary configuration for SPI communication on STM32 devices.

### bos1901_dummy_spi_dev_config_t
- **Description**: This is a dummy configuration for SPI devices, used for testing and as a placeholder. It contains a generic handle and provides a weakly defined interface for SPI operations.
- **Usage**: Used for testing and documentation purposes. Provides a fallback implementation that can be overridden.