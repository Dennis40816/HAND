/**
 * Good Reference:
 * 1. How ROI set works: we set upper left and lower right corner coordinate
 * (ranging from (0,0) to (15, 15))
 * https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library/issues/9#issuecomment-419504574
 *
 * 2. Port to ESPIDF
 * https://blog.csdn.net/qq_20515461/article/details/99293706
 *
 * 3. https://blog.csdn.net/tiramisu_L/article/details/90729964
 */

/**
 * Key function
 *
 * 1. calibrtion
 *  -
 * 2. measurement
 */

#include "hand_common.h"

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "vl53l1x.h"

/* macros */
#define VL53L1X_1_INDEX    1
#define VL53L1X_2_INDEX    2
#define VL53L1X_XSHUT_HIGH 1
#define VL53L1X_XSHUT_LOW  0

#define VL53L1X_1_NEW_I2C_ADDRESS   0x60
#define VL53L1X_2_NEW_I2C_ADDRESS   0x62
#define VL53L1X_DEFAULT_I2C_ADDRESS 0x52

#define VL53L1X_MODEL_ID_INDEX      0x010F
#define VL53L1X_MODULE_TYPE_INDEX   0x0110
#define VL53L1X_MASK_REVISION_INDEX 0x0111

static const char* TAG = "VL53L1X_GROUP_TEST";

/* create vl53l1x devices */
VL53L1_Dev_t vl53l1x_1 = {.I2cHandle = I2C_NUM_0,
                          .comms_speed_khz = (uint16_t)400,
                          .I2cDevAddr = VL53L1X_DEFAULT_I2C_ADDRESS,
                          .new_data_ready_poll_duration_ms = 100};

VL53L1_Dev_t vl53l1x_2 = {.I2cHandle = I2C_NUM_0,
                          .comms_speed_khz = (uint16_t)400,
                          .I2cDevAddr = VL53L1X_DEFAULT_I2C_ADDRESS,
                          .new_data_ready_poll_duration_ms = 100};

void i2c_bus_init()
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

static uint8_t vl53l1x_read_register(uint8_t i2c_address, uint16_t reg_addr)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  uint8_t data = 0;

  /* write part */
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, i2c_address | I2C_MASTER_WRITE, true);
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
  i2c_master_write_byte(cmd, i2c_address | I2C_MASTER_READ, true);
  i2c_master_read_byte(cmd, &data, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
  i2c_cmd_link_delete(cmd);

  ESP_LOGI(TAG,
           "I2C addr @{0x%02X} Status {%d}. Read 0x%02X from register 0x%04X",
           i2c_address, ret, data, reg_addr);
  return data;
}

/* remain xshut high after call this function */
static void vl53l1x_change_i2c_address()
{
  VL53L1_GpioXshutdown(VL53L1X_1_INDEX, VL53L1X_XSHUT_LOW);
  VL53L1_GpioXshutdown(VL53L1X_2_INDEX, VL53L1X_XSHUT_LOW);

  /* change VL53L1X_1 i2c address to 0x60 */
  VL53L1_GpioXshutdown(VL53L1X_1_INDEX, VL53L1X_XSHUT_HIGH);
  VL53L1X_SetI2CAddress(&vl53l1x_1, VL53L1X_1_NEW_I2C_ADDRESS);

  /* change VL53L1X_2 i2c address to 0x62 */
  VL53L1_GpioXshutdown(VL53L1X_2_INDEX, VL53L1X_XSHUT_HIGH);
  VL53L1X_SetI2CAddress(&vl53l1x_2, VL53L1X_2_NEW_I2C_ADDRESS);
}

void vl53l1x_init(VL53L1_DEV dev) {

}

void app_main(void)
{
  esp_log_level_set("*", ESP_LOG_DEBUG);
  vTaskDelay(pdMS_TO_TICKS(3000));

  i2c_bus_init();

  VL53L1_ConfigTca6408a(I2C_NUM_0);

  /* change i2c address */
  vl53l1x_change_i2c_address();

  /* try to read vl53l1x_1 */
  uint8_t model_id = 0x00;
  VL53L1_RdByte(&vl53l1x_1, VL53L1X_MODEL_ID_INDEX, &model_id);

  ESP_LOGI(TAG, "Reading from address: {0x%02X}", vl53l1x_1.I2cDevAddr);
  ESP_LOGI(TAG, "Model ID is: {0x%02X}", model_id);

  /* try to read vl53l1x_2 */
  model_id = 0x00;

  VL53L1_RdByte(&vl53l1x_2, VL53L1X_MODEL_ID_INDEX, &model_id);

  ESP_LOGI(TAG, "Reading from address: {0x%02X}", vl53l1x_2.I2cDevAddr);
  ESP_LOGI(TAG, "Model ID is: {0x%02X}", model_id);
}