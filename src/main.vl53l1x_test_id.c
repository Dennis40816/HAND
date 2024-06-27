#include "hand_common.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "tca6408a.h"

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

  /* write part */
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (VL_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, (reg_addr >> 8) & 0xFF,
                        true);  // Send high byte of register address
  i2c_master_write_byte(cmd, reg_addr & 0xFF,
                        true);  // Send low byte of register address
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);

  /* Note: do not combine read part and write part together, which cause error
   */

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Read failed in write stage");
    return 0;
  }

  cmd = i2c_cmd_link_create();

  /* read part */
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

/* VL53L1X use I2C Bus 0 */
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

  /* WARNING: we disable the i2c master interrupt (which samg55 enable) */
  /* See i2c_master_register_event_callbacks() for more information */
  ESP_ERROR_CHECK(i2c_driver_install(i2c0_port, i2c0_config.mode, 0, 0, 0));
}

void app_main(void)
{
  vTaskDelay(pdMS_TO_TICKS(3000));
  ESP_LOGI(TAG, "Start to init i2c bus 0");
  i2c_bus0_init();

  /* raise XSHUT 1 of VL53L1X_1 */
  tca6408a_dev_t tca6408a = {.address = 0x20, .i2c_bus = I2C_NUM_0};

  tca6408a_set_pin_output_mode(&tca6408a, VL_XSHUT_1_PIN);
  tca6408a_set_pin_output_mode(&tca6408a, VL_XSHUT_2_PIN);
  tca6408a_set_pin_low(&tca6408a, VL_XSHUT_1_PIN);
  tca6408a_set_pin_low(&tca6408a, VL_XSHUT_2_PIN);

  /* check device 1 */
  ESP_LOGI(TAG, "Device 1 starts!");
  tca6408a_set_pin_high(&tca6408a, VL_XSHUT_1_PIN);

  tca6408a_reg_info_t reg_info = {0};
  tca6408a_read_all(&tca6408a, &reg_info);
  char s[120];
  tca6408a_reg_info_to_str(&reg_info, s, 120);

  ESP_LOGI(TAG, "\n%s", s);

  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "Wait for 100 ms");

  vl53l1x_read_register(VL_MODEL_ID_INDEX);
  vl53l1x_read_register(VL_MODULE_TYPE_INDEX);
  vl53l1x_read_register(VL_MASK_REVISION_INDEX);
  tca6408a_set_pin_low(&tca6408a, VL_XSHUT_1_PIN);

  /* check device 2 */
  ESP_LOGI(TAG, "Device 2 starts!");
  tca6408a_set_pin_high(&tca6408a, VL_XSHUT_2_PIN);

  tca6408a_read_all(&tca6408a, &reg_info);
  tca6408a_reg_info_to_str(&reg_info, s, 120);

  ESP_LOGI(TAG, "\n%s", s);

  vTaskDelay(pdMS_TO_TICKS(100));
  ESP_LOGI(TAG, "Wait for 100 ms");

  vl53l1x_read_register(VL_MODEL_ID_INDEX);
  vl53l1x_read_register(VL_MODULE_TYPE_INDEX);
  vl53l1x_read_register(VL_MASK_REVISION_INDEX);
  tca6408a_set_pin_low(&tca6408a, VL_XSHUT_2_PIN);
}