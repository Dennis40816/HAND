#ifndef LIB_TCA6408A_H_
#define LIB_TCA6408A_H_

#include <stdint.h>

#include "driver/i2c.h"
#include "esp_err.h"

#define TCA6408A_DEFAULT_I2C_ADDRESS 0x20

// TCA6408A Register addresses
#define TCA6408A_INPUT_REG    0x00
#define TCA6408A_OUTPUT_REG   0x01
#define TCA6408A_POLARITY_REG 0x02
#define TCA6408A_CONFIG_REG   0x03

/* Config macros */
#define TCA6408A_CONFIG_OUTPUT (0)
#define TCA6408A_CONFIG_INPUT  (1)

#ifdef ESP_PLATFORM
typedef i2c_port_t i2c_port_type;
#else
typedef i2c_port_t int;
#endif

typedef struct {
  i2c_port_type i2c_port;
} tca6408a_t;

esp_err_t tca6408a_write_register(uint8_t reg, uint8_t value,
                                  tca6408a_t* config);
esp_err_t tca6408a_read_register(uint8_t reg, uint8_t* value,
                                 tca6408a_t* config);
esp_err_t tca6408a_set_input(uint8_t value, tca6408a_t* config);
esp_err_t tca6408a_set_output(uint8_t value, tca6408a_t* config);
esp_err_t tca6408a_set_low(uint8_t mask, tca6408a_t* config);
esp_err_t tca6408a_set_high(uint8_t value, uint8_t mask, tca6408a_t* config);

#endif