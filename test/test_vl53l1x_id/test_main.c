#include "driver/gpio.h"
#include "driver/i2c.h"
#include "tca6408a.h"
#include "unity.h"

static const char* TAG = "HAND_VL53L1X_TEST";

#define VL_I2C_READ 0x1  // LSB set
#define VL_I2C_ADDR (0x52 >> 1)

#define VL_MODEL_ID_INDEX      0x010F
#define VL_MODULE_TYPE_INDEX   0x0110
#define VL_MASK_REVISION_INDEX 0x0111

#define VL_XSHUT_1_PIN 3
#define VL_XSHUT_2_PIN 5

uint8_t vl53l1x_read_register(uint16_t reg_addr)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  uint8_t data = 0;

  // Write part
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (VL_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, (reg_addr >> 8) & 0xFF,
                        true);  // Send high byte of register address
  i2c_master_write_byte(cmd, reg_addr & 0xFF,
                        true);  // Send low byte of register address
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Read failed in write stage");
    return 0;
  }

  cmd = i2c_cmd_link_create();

  // Read part
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (VL_I2C_ADDR << 1) | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);

  ESP_LOGI(TAG, "Status {%d}. Read 0x%02X from register 0x%04X", ret, data,
           reg_addr);
  return data;
}

void i2c_bus0_init()
{
  const i2c_port_t i2c0_port = I2C_NUM_0;
  i2c_config_t i2c0_config = {.mode = I2C_MODE_MASTER,
                              .sda_io_num = GPIO_NUM_1,
                              .scl_io_num = GPIO_NUM_0,
                              .sda_pullup_en = GPIO_PULLUP_DISABLE,
                              .scl_pullup_en = GPIO_PULLUP_DISABLE,
                              .master.clk_speed = 400000};

  ESP_ERROR_CHECK(i2c_param_config(i2c0_port, &i2c0_config));
  ESP_ERROR_CHECK(i2c_driver_install(i2c0_port, i2c0_config.mode, 0, 0, 0));
}

void setUp(void)
{
  vTaskDelay(pdMS_TO_TICKS(3000));
  ESP_LOGI(TAG, "Start to init i2c bus 0");
  i2c_bus0_init();
  tca6408a_dev_t tca6408a = {.address = 0x20, .i2c_bus = I2C_NUM_0};

  tca6408a_set_pin_output_mode(&tca6408a, VL_XSHUT_1_PIN);
  tca6408a_set_pin_output_mode(&tca6408a, VL_XSHUT_2_PIN);
  tca6408a_set_pin_low(&tca6408a, VL_XSHUT_1_PIN);
  tca6408a_set_pin_low(&tca6408a, VL_XSHUT_2_PIN);
}

void tearDown(void) { i2c_driver_delete(I2C_NUM_0); }

void test_vl53l1x_registers(uint8_t device_pin)
{
  tca6408a_dev_t tca6408a = {.address = 0x20, .i2c_bus = I2C_NUM_0};

  // Activate the device
  tca6408a_set_pin_high(&tca6408a, device_pin);

  vTaskDelay(pdMS_TO_TICKS(100));  // Wait for 100 ms

  uint8_t model_id = vl53l1x_read_register(VL_MODEL_ID_INDEX);
  uint8_t module_type = vl53l1x_read_register(VL_MODULE_TYPE_INDEX);
  uint8_t mask_revision = vl53l1x_read_register(VL_MASK_REVISION_INDEX);

  TEST_ASSERT_EQUAL_HEX8(0xEA, model_id);
  TEST_ASSERT_EQUAL_HEX8(0xCC, module_type);
  TEST_ASSERT_EQUAL_HEX8(0x10, mask_revision);

  // Deactivate the device
  tca6408a_set_pin_low(&tca6408a, device_pin);
}

void test_vl53l1x_device_1(void) { test_vl53l1x_registers(VL_XSHUT_1_PIN); }

void test_vl53l1x_device_2(void) { test_vl53l1x_registers(VL_XSHUT_2_PIN); }

void app_main(void)
{
  UNITY_BEGIN();
  RUN_TEST(test_vl53l1x_device_1);
  RUN_TEST(test_vl53l1x_device_2);
  UNITY_END();
}
