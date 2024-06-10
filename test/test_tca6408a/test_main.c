#include <unity.h>
#include "tca6408a.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

// I2C Master Port Number
#define I2C_MASTER_NUM I2C_NUM_0

// I2C Master Configuration
#define I2C_MASTER_FREQ_HZ        400000  // I2C Master Clock Frequency
#define I2C_MASTER_TX_BUF_DISABLE 0       // I2C Master Do Not Need Buffer
#define I2C_MASTER_RX_BUF_DISABLE 0       // I2C Master Do Not Need Buffer
#define I2C_MASTER_TIMEOUT_MS     1000    // I2C Master Timeout

#define I2C_SDA_GPIO GPIO_NUM_1
#define I2C_SCL_GPIO GPIO_NUM_0

static const char *TAG = "test_tca6408a";

// Define a structure to hold test parameters
typedef struct
{
  tca6408a_reg_info_t reg_info_a;
  tca6408a_reg_info_t reg_info_b;
} test_params_t;

// Global variable to store test parameters
test_params_t g_test_params;

esp_err_t i2c_master_init(gpio_num_t scl_pin, gpio_num_t sda_pin)
{
  // Configuration structure for the I2C master driver
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = sda_pin,
      .sda_pullup_en = GPIO_PULLUP_DISABLE,
      .scl_io_num = scl_pin,
      .scl_pullup_en = GPIO_PULLUP_DISABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };

  // Install the I2C driver
  esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
  if (err != ESP_OK)
  {
    return err;
  }

  // Install the I2C driver with the specified configuration
  return i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                            I2C_MASTER_RX_BUF_DISABLE,
                            I2C_MASTER_TX_BUF_DISABLE, 0);
}

void setUp(void)
{
  // Initialize global test parameters before each test
}

void tearDown(void)
{
  // Clean up after each test, if necessary
  g_test_params.reg_info_a.input_reg = 0;
  g_test_params.reg_info_a.output_reg = 0;
  g_test_params.reg_info_a.polarity_reg = 0;
  g_test_params.reg_info_a.config_reg = 0;
}

void test_tca6408a_default_value_a(void)
{
  // Check the values read from the sensor
  TEST_ASSERT_EQUAL_UINT8(0b11111111, g_test_params.reg_info_a.output_reg);
  TEST_ASSERT_EQUAL_UINT8(0b00000000, g_test_params.reg_info_a.polarity_reg);
  TEST_ASSERT_EQUAL_UINT8(0b11111111, g_test_params.reg_info_a.config_reg);
}

void test_tca6408a_default_value_b(void)
{
  // Check the values read from the sensor
  TEST_ASSERT_EQUAL_UINT8(0b11111111, g_test_params.reg_info_b.output_reg);
  TEST_ASSERT_EQUAL_UINT8(0b00000000, g_test_params.reg_info_b.polarity_reg);
  TEST_ASSERT_EQUAL_UINT8(0b11111111, g_test_params.reg_info_b.config_reg);
}

void test_tca6408a_set_p3_output_high(void) {}

void app_main(void)
{
  UNITY_BEGIN();
  /* wait until PC ready */
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Initialize I2C master with GPIO21 as SDA and GPIO22 as SCL
  esp_err_t ret = i2c_master_init(I2C_SCL_GPIO, I2C_SDA_GPIO);
  if (ret == ESP_OK)
  {
    ESP_LOGI(TAG, "I2C initialized successfully\n");
  }
  else
  {
    ESP_LOGE(TAG, "Failed to initialize I2C\n");
    return;
  }

  // Initialize the TCA6408A device
  tca6408a_dev_t tca6408a_dev_a = {.address = 0x20, .i2c_bus = I2C_MASTER_NUM};

  // Read all registers from the TCA6408A device (0x20)
  ret = tca6408a_read_all(&tca6408a_dev_a, &g_test_params.reg_info_a);
  if (ret == ESP_OK)
  {
    ESP_LOGI(TAG, "TCA6408A @0x20 registers read successfully\n");
    char tmp[128];
    const int max_length = 128;
    ret = tca6408a_reg_info_to_str(&g_test_params.reg_info_a, tmp, max_length);
    if (ret == ESP_OK)
    {
      ESP_LOGI(TAG, "\n%s", tmp);
    }
    else
    {
      ESP_LOGW(TAG, "tca6408a_reg_info_to_str has failed!");
    }
  }
  else
  {
    ESP_LOGE(TAG, "Failed to read TCA6408A @0x20 registers\n");
    return;
  }

  // Try to read Another TCA6408A
  tca6408a_dev_t tca6408a_dev_b = {.address = 0x21, .i2c_bus = I2C_MASTER_NUM};
  // Read all registers from the TCA6408A device (0x20)
  ret = tca6408a_read_all(&tca6408a_dev_b, &g_test_params.reg_info_b);
  if (ret == ESP_OK)
  {
    ESP_LOGI(TAG, "TCA6408A @0x21 registers read successfully\n");
    char tmp[128];
    const int max_length = 128;
    ret = tca6408a_reg_info_to_str(&g_test_params.reg_info_b, tmp, max_length);
    if (ret == ESP_OK)
    {
      ESP_LOGI(TAG, "\n%s", tmp);
    }
    else
    {
      ESP_LOGW(TAG, "tca6408a_reg_info_to_str has failed!");
    }
  }
  else
  {
    ESP_LOGE(TAG, "Failed to read TCA6408A @0x21 registers\n");
    return;
  }

  // Run tests
  RUN_TEST(test_tca6408a_default_value_a);
  RUN_TEST(test_tca6408a_default_value_b);

  // Add more test here

  UNITY_END();
}
