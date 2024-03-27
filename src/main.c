#ifdef __cplusplus
extern "C" {
#endif
void app_main(void);
#ifdef __cplusplus
}
#endif

/* esp-idf include */
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* config include */
#include "board_config.h"

/* test include */
#include "drivers/chirpmicro/inc/chirp_bsp.h"
#include "miniz.h"
/* end test include */

#include "tca6408a.h"

#define I2C_OTHER_MASTER_SCL_IO 5  /*!< gpio number for I2C master clock */
#define I2C_OTHER_MASTER_SDA_IO 4  /*!< gpio number for I2C master data  */
#define I2C_CH101_MASTER_SCL_IO 13 /*!< gpio number for I2C master clock */
#define I2C_CH101_MASTER_SDA_IO 12 /*!< gpio number for I2C master data  */
#define I2C_MASTER_OTHER I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_CH101 I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 400000 /*!< I2C master clock frequency */

#define GPIO_TCA_INT 6   /*!< TCA Interrupt GPIO */
#define GPIO_TCA_RESET 0 /*!< TCA Reset GPIO */
#define GPIO_TCA_ADDR 1  /*!< TCA Address GPIO */

#define GPIO_CH101_INT 7 /*!< CH101 Interrupt GPIO */

#define TCA_CH101_RESET (1 << 6)
#define TCA_CH101_PROG (1 << 7)
#define TCA_CH101_INT (1 << 5)

int i = 0;

static void i2c_master_init(void) {
  // Configure I2C for "other" device
  i2c_config_t conf_other = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_OTHER_MASTER_SDA_IO,
      .scl_io_num = I2C_OTHER_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  i2c_param_config(I2C_MASTER_OTHER, &conf_other);
  i2c_driver_install(I2C_MASTER_OTHER, conf_other.mode, 0, 0, 0);

  // Configure I2C for CH101 device
  // Note: If they need to be on different I2C ports, you will need to adjust
  // I2C_MASTER_NUM and initialization accordingly.
  i2c_config_t conf_ch101 = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_CH101_MASTER_SDA_IO,
      .scl_io_num = I2C_CH101_MASTER_SCL_IO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ,
  };
  // Assuming both I2C on the same I2C_NUM_0 for simplicity, otherwise use
  // I2C_NUM_1 for the second
  i2c_param_config(I2C_MASTER_CH101, &conf_ch101);
  i2c_driver_install(I2C_MASTER_CH101, conf_ch101.mode, 0, 0, 0);
}

/**
 * GPIO interrupt callback functions
 */
static void IRAM_ATTR tca_interrupt_cb(void* arg) {
  ++i;
  // the INT should be reset automatively
}

static void IRAM_ATTR ch101_interrupt_cb(void* arg) {
  // Handle CH101 interrupt
}

static void compile_test(void) { chbsp_delay_ms(50); }

static void gpio_init(void) {
  gpio_config_t io_conf;

  // Configure TCA_INT as input, pull up, with interrupt falling edge
  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << GPIO_TCA_INT);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);
  gpio_install_isr_service(0);
  gpio_isr_handler_add(GPIO_TCA_INT, tca_interrupt_cb, (void*)GPIO_TCA_INT);

  // Configure TCA_RESET as output, always high
  gpio_set_direction(GPIO_TCA_RESET, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_TCA_RESET, 1);

  // Configure TCA_ADDR as output, always low
  gpio_set_direction(GPIO_TCA_ADDR, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_TCA_ADDR, 0);

  // Configure CH101_INT (TXB0104_B4) as input/output, pull low, with
  // interrupt with rising edge Note: GPIO_MODE_INPUT_OUTPUT_OD for open-drain
  // if required by your application
  // TODO: Check this
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << GPIO_CH101_INT);
  io_conf.pull_down_en = 1;
  io_conf.pull_up_en = 0;
  gpio_config(&io_conf);
  gpio_isr_handler_add(GPIO_CH101_INT, ch101_interrupt_cb,
                       (void*)GPIO_CH101_INT);
}

static void tca6408a_init() {
  tca6408a_set_output(TCA_CH101_RESET | TCA_CH101_PROG);
  tca6408a_set_input(TCA_CH101_INT);
}

void isr_monitor_task(void* para) {
  static int i_cache = 0;
  for (;;) {
    if (i_cache != i) {
      ESP_LOGW("isr_monitor_task", "i is %d", i);
      i_cache = i;
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void app_main(void) {
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI("app_main", "Run init");
  /*

  board_init();
    // bus_init();
    // gpio_init()
    // ble_init();
    // wifi_init();
  devices_init(); // internal program..., read config
    // tca6408a * 2
    // ch101 * 4
    // imu * 1
    // kx132-1211 * 4
    // bos1901 * 4
    // battery gauge * 1
    //

  task_init();

  // wait for connection

   */
  i2c_master_init();
  gpio_init();

  tca6408a_init();

  compile_test();

  /* create test */
  xTaskCreate(&isr_monitor_task, "isr_monitor_task", 4096, NULL, 3, NULL);
}
