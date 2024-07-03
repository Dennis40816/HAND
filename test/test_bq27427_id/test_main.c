#include <unity.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char *TAG = "BQ27427_ID_TEST";

#define BQ_I2C_SCL_PIN GPIO_NUM_0
#define BQ_I2C_SDA_PIN GPIO_NUM_1
#define BQ_I2C_BUS     I2C_NUM_0

#define BQ274XX_CMD_CONTROL      0x00
#define BQ274XX_CTRL_DEVICE_TYPE 0x0001
#define BQ274XX_I2C_ADDR         0x55

void i2c_bus_init()
{
  i2c_config_t i2c_config = {.mode = I2C_MODE_MASTER,
                             .scl_pullup_en = GPIO_PULLUP_DISABLE,
                             .sda_pullup_en = GPIO_PULLUP_DISABLE,
                             .scl_io_num = BQ_I2C_SCL_PIN,
                             .sda_io_num = BQ_I2C_SDA_PIN,
                             .master.clk_speed = 400000};

  TEST_ASSERT_EQUAL(ESP_OK, i2c_param_config(BQ_I2C_BUS, &i2c_config));
  TEST_ASSERT_EQUAL(ESP_OK,
                    i2c_driver_install(BQ_I2C_BUS, i2c_config.mode, 0, 0, 0));

  const int factor = 18;
  i2c_set_timeout(BQ_I2C_BUS, factor);
}

static esp_err_t bq274xx_ctrl_reg_write(uint16_t subcommand)
{
  uint8_t tx_buf[3] = {BQ274XX_CMD_CONTROL, (uint8_t)(subcommand & 0xFF),
                       (uint8_t)((subcommand >> 8) & 0xFF)};

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (BQ274XX_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write(cmd, tx_buf, sizeof(tx_buf), true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(BQ_I2C_BUS, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);

  return ret;
}

static esp_err_t bq274xx_cmd_reg_read(uint8_t reg_addr, uint16_t *val)
{
  uint8_t i2c_data[2];

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (BQ274XX_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, reg_addr, true);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (BQ274XX_I2C_ADDR << 1) | I2C_MASTER_READ, true);
  i2c_master_read(cmd, i2c_data, sizeof(i2c_data), I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(BQ_I2C_BUS, cmd, pdMS_TO_TICKS(1000));
  i2c_cmd_link_delete(cmd);

  if (ret == ESP_OK)
  {
    *val = (uint16_t)((i2c_data[1] << 8) | i2c_data[0]);
  }

  return ret;
}

static esp_err_t bq274xx_get_device_type(uint16_t *val)
{
  esp_err_t ret = bq274xx_ctrl_reg_write(BQ274XX_CTRL_DEVICE_TYPE);
  if (ret != ESP_OK)
  {
    return ret;
  }

  return bq274xx_cmd_reg_read(BQ274XX_CMD_CONTROL, val);
}

void test_bq27427_device_type()
{
  uint16_t device_type;
  TEST_ASSERT_EQUAL(ESP_OK, bq274xx_get_device_type(&device_type));
  ESP_LOGI(TAG, "Device type: 0x%04x", device_type);
  TEST_ASSERT_EQUAL_HEX16(0x0427, device_type);
}

void setUp() { i2c_bus_init(); }

void tearDown() { i2c_driver_delete(BQ_I2C_BUS); }

void app_main()
{
  vTaskDelay(pdMS_TO_TICKS(3000));
  UNITY_BEGIN();
  RUN_TEST(test_bq27427_device_type);
  UNITY_END();
}