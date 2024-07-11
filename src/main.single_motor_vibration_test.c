#include "hand_common.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/semphr.h"
#include "bos1901.h"

#include "math.h"

static SemaphoreHandle_t xMutex = NULL;

/**
 * @brief See details in src/history/main.motor_output_single_value_test
 *
 */
#define REFERENCE_MINUS_1LSB 0x0FFF
#define REFERENCE_PLUS_1LSB  0x0001

#define PIN_NUM_MISO  11
#define PIN_NUM_MOSI  13
#define PIN_NUM_CLK   12
#define PIN_NUM_CS0   10
#define PIN_NUM_CS1   9
#define PIN_NUM_CS2   8
#define PIN_NUM_CS3   7
#define PIN_KX_CS     6
#define PIN_BMI_CS    5
#define PIN_LED_RED   43
#define PIN_LED_WHITE 44
#define HSPI_HOST     (SPI2_HOST)

static const char *TAG = "HAND_BOS1901_MOTOR_TEST";

spi_device_handle_t spi_handle_0;
spi_device_handle_t spi_handle_1;
spi_device_handle_t spi_handle_2;
spi_device_handle_t spi_handle_3;

bos1901_dev_t *bos1901_device_0;
bos1901_dev_t *bos1901_device_1;
bos1901_dev_t *bos1901_device_2;
bos1901_dev_t *bos1901_device_3;

#define BOS1901_TEST_DEVICE bos1901_device_0

#define SINE_FREQ      (200)
#define WAVEFORM_SIZE  (8000 / SINE_FREQ)
#define SPI_QUEUE_SIZE (100)
#define SPI_SPEED_M    (30)

/* calculated waveform */
uint16_t waveform[WAVEFORM_SIZE + 1];

/* Math helper */
static int16_t volt_2_amp(float volt)
{
  int16_t amplitude = volt * 2047 / 3.6 / 31;
  return amplitude & 0x0FFF;
}

/* Math helper */
static float amp_to_volt(int16_t amp)
{
  // 先反轉之前的轉換過程
  float volt = (amp & 0x0FFF) * 3.6 * 31 / 2047;
  return volt;
}

static void calculate_waveform(uint16_t *table, uint16_t *size, float v_max,
                               float v_min, uint16_t freq, uint8_t cycles)
{
  const uint16_t sampling_rate = 8000;

  float amplitude = (v_max - v_min) / 2;
  float offset = (v_max + v_min) / 2;

  uint16_t sample_per_cycle = round(sampling_rate / (float)freq);

  ESP_LOGD(TAG, "sample_per_cycle: %d", sample_per_cycle);

  double theta0 = 2 * M_PI / sample_per_cycle;  // theta unit

  float phase_shift;
  uint16_t end_value;

  if (v_min >= 0)
  {
    phase_shift = -M_PI;
    end_value = REFERENCE_MINUS_1LSB;
  }
  else if (v_max <= 0)
  {
    phase_shift = 0;
    end_value = REFERENCE_PLUS_1LSB;
  }
  else
  {
    phase_shift = -M_PI - acosf(fabsf(offset) / amplitude);
    end_value = fabsf(v_max) > fabsf(v_min) ? REFERENCE_MINUS_1LSB
                                            : REFERENCE_PLUS_1LSB;
  }

  *size = sample_per_cycle * cycles;

  for (uint16_t i = 0; i < *size; ++i)
  {
    float tmp = (v_max - v_min) / 2 * cos((double)(theta0 * i + phase_shift)) +
                (v_max + v_min) / 2;
    // ESP_LOGI(TAG, "%.3f", tmp);
    table[i] = volt_2_amp(tmp);
  }

  /* This part for sensing only! */
  // (*size)++;
  // table[(*size) - 1] = end_value;
}

/* TODO, enable and disable is controlled by app_main task */
static void fill_fifo_to_bos1901_0_task(void *para)
{
  uint16_t waveform_size;

  // WARNING: must not exceed 60 V
  calculate_waveform(waveform, &waveform_size, 60.0f, 10.0f, SINE_FREQ, 1);

  ESP_LOGI(TAG, "waveform_size is: %d", waveform_size);

  volatile uint16_t ic_status = 0;
  volatile uint16_t next_start_index = 0;

  while (1)
  {
    if (xMutex != NULL)
    {
      // 嘗試獲取互斥鎖
      if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
      {
        // 讀取 ic_status 寄存器
        bos1901_device_read_reg(BOS1901_TEST_DEVICE, BOS1901_REG_IC_STATUS,
                                &ic_status);

        // 判斷 FIFO 是否為空或已滿
        volatile bool fifo_is_empty = ic_status & (1 << 6);
        volatile bool fifo_is_full = ic_status & (1 << 7);
        volatile uint16_t fifo_space = ic_status & 0b111111;

        if (fifo_is_empty)
        {
          // 傳送第一段
          uint32_t len = WAVEFORM_SIZE > 64 ? 64 : WAVEFORM_SIZE;
          bos1901_device_read_write(BOS1901_TEST_DEVICE, waveform, NULL, len);
          if (WAVEFORM_SIZE > 64)
          {
            next_start_index = 64;
          }
        }
        else if (fifo_is_full)
        {
          // wait fifo
          // 就算沒有這個也不行，所以不是這個的問題
        }
        else
        {
          // 本版本對於 125Hz 以上運作良好，如果接上電腦沒辦法震動，大概率是
          // power 不夠，請直接用接頭接到牆上，但對於低於 125 Hz 存在 bug。

          // FIXME: 問題應該在這下面，待修正:
          // 推測可能是某種操作導致 BOS1901 無法正確接收傳入 FIFO
          // 的值，因為當一開始設定成 100 Hz(有 bug)後換成
          // 125Hz，是無法震動的，但重新上電(reset
          // bos1901)後就可以震動，因此推測 BOS1901 在 125Hz
          // 因不明原因可能被寫壞掉了。

          // - 重讀數據手冊
          // - 可以寫一個模擬器來模擬任意的 FIFO 回傳值，確認該 function
          // 不存在問題
          // - 要用 LA 分析 SPI
          // - 可以讀取看看錯誤 register
          // - 可以減少寫入數量，不一定要寫滿
          // - 更多請查看 BOS1901 馬達測試 notion

          // - 內部時鐘的問題嗎?
          uint16_t array_remain_sample = waveform_size - next_start_index;

          if (array_remain_sample > fifo_space)
          {
            // 寫入 fifo_space bytes 到 FIFO
            bos1901_device_read_write(BOS1901_TEST_DEVICE,
                                      &waveform[next_start_index], NULL,
                                      fifo_space);
            next_start_index += fifo_space;  // 更新索引
          }
          else
          {
            // 寫入剩餘樣本到 FIFO
            bos1901_device_read_write(BOS1901_TEST_DEVICE,
                                      &waveform[next_start_index], NULL,
                                      array_remain_sample);
            next_start_index = 0;  // 重置索引

            uint16_t rest_space = fifo_space - array_remain_sample;

            // 如果還有剩餘空間，寫入剩餘 bytes
            bos1901_device_read_write(BOS1901_TEST_DEVICE, waveform, NULL,
                                      rest_space);
            next_start_index += rest_space;
          }
        }

        xSemaphoreGive(xMutex);
        vTaskDelay(pdMS_TO_TICKS(1));
      }
    }
  }
}

/* Other private functions */

static void init_spi_bus()
{
  spi_bus_config_t buscfg = {
      .miso_io_num = PIN_NUM_MISO,
      .mosi_io_num = PIN_NUM_MOSI,
      .sclk_io_num = PIN_NUM_CLK,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 4096,
  };

  esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
  ESP_LOGI(TAG, "spi bus initialize ret value is: %d. should be 0", ret);
}

static void init_bos1901_device(bos1901_dev_t **device, const char *name,
                                spi_device_handle_t *handle,
                                spi_device_interface_config_t *devcfg)
{
  bos1901_spi_dev_config_t espidf_device_config = {
      .target_bus = SPI2_HOST,
      .handle_ptr = handle,
      .dev_config = *devcfg,
  };

  *device = bos1901_device_create(name);
  bos1901_device_init(*device, NULL, &espidf_device_config);
}

static void init_spi()
{
  init_spi_bus();

  spi_device_interface_config_t devcfg0 = {
      .clock_speed_hz = SPI_SPEED_M * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_NUM_CS0,
      .queue_size = SPI_QUEUE_SIZE,
  };

  spi_device_interface_config_t devcfg1 = {
      .clock_speed_hz = SPI_SPEED_M * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_NUM_CS1,
      .queue_size = SPI_QUEUE_SIZE,
  };

  spi_device_interface_config_t devcfg2 = {
      .clock_speed_hz = SPI_SPEED_M * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_NUM_CS2,
      .queue_size = SPI_QUEUE_SIZE,
  };

  spi_device_interface_config_t devcfg3 = {
      .clock_speed_hz = SPI_SPEED_M * 1000 * 1000,
      .mode = 0,
      .spics_io_num = PIN_NUM_CS3,
      .queue_size = SPI_QUEUE_SIZE,
  };

  init_bos1901_device(&bos1901_device_0, "BOS1901_Device_0", &spi_handle_0,
                      &devcfg0);
  init_bos1901_device(&bos1901_device_1, "BOS1901_Device_1", &spi_handle_1,
                      &devcfg1);
  init_bos1901_device(&bos1901_device_2, "BOS1901_Device_2", &spi_handle_2,
                      &devcfg2);
  init_bos1901_device(&bos1901_device_3, "BOS1901_Device_3", &spi_handle_3,
                      &devcfg3);
}

static void init_led()
{
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pin_bit_mask = (1ULL << PIN_LED_RED) | (1ULL << PIN_LED_WHITE);
  gpio_config(&io_conf);

  gpio_set_level(PIN_LED_RED, 0);
  gpio_set_level(PIN_LED_WHITE, 0);
}

static void set_other_cs_pins_high()
{
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pin_bit_mask = ((1ULL << PIN_KX_CS) | (1ULL << PIN_BMI_CS));

  gpio_config(&io_conf);

  gpio_set_level(PIN_KX_CS, 1);
  gpio_set_level(PIN_BMI_CS, 1);

  ESP_LOGI("GPIO", "GPIO %d (KX132-1211 CS) set to output high", PIN_KX_CS);
  ESP_LOGI("GPIO", "GPIO %d (BMI323 CS) set to output high", PIN_BMI_CS);
}

static void test_bos1901_device()
{
  set_other_cs_pins_high();
  init_led();

  bos1901_device_reset(bos1901_device_0);
  bos1901_device_reset(bos1901_device_1);
  bos1901_device_reset(bos1901_device_2);
  bos1901_device_reset(bos1901_device_3);

  vTaskDelay(pdMS_TO_TICKS(50));

  uint16_t expected_value = 0x246A;
  uint16_t read_value_0, read_value_1, read_value_2, read_value_3;

  bos1901_device_read_reg(bos1901_device_0, BOS1901_REG_ID, &read_value_0);
  bos1901_device_read_reg(bos1901_device_1, BOS1901_REG_ID, &read_value_1);
  bos1901_device_read_reg(bos1901_device_2, BOS1901_REG_ID, &read_value_2);
  bos1901_device_read_reg(bos1901_device_3, BOS1901_REG_ID, &read_value_3);

  ESP_LOGI(TAG, "Read_value_0: 0x%04X, expected: 0x%04X", read_value_0,
           expected_value);
  ESP_LOGI(TAG, "Read_value_1: 0x%04X, expected: 0x%04X", read_value_1,
           expected_value);
  ESP_LOGI(TAG, "Read_value_2: 0x%04X, expected: 0x%04X", read_value_2,
           expected_value);
  ESP_LOGI(TAG, "Read_value_3: 0x%04X, expected: 0x%04X", read_value_3,
           expected_value);
}

static void configure_hand_bos1901_test()
{
  uint16_t kp_reg_val;
  bos1901_device_read_reg(BOS1901_TEST_DEVICE, BOS1901_REG_KP, &kp_reg_val);
  kp_reg_val &= 0xFFF;
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "Got KP REG value: 0x%04X. EXPECTED VALUE is {0x0080}",
           kp_reg_val);

  uint16_t parcap_reg_val;
  bos1901_device_read_reg(BOS1901_TEST_DEVICE, BOS1901_REG_PARCAP,
                          &parcap_reg_val);
  parcap_reg_val &= 0xFFF;
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "Got PARCAP REG value: 0x%04X. EXPECTED VALUE is {0x073A}",
           parcap_reg_val);

  uint16_t config_reg_val;
  bos1901_device_read_reg(BOS1901_TEST_DEVICE, BOS1901_REG_CONFIG,
                          &config_reg_val);
  config_reg_val &= 0xFFF;
  vTaskDelay(pdMS_TO_TICKS(1000));
  ESP_LOGI(TAG, "Got CONFIG REG value: 0x%04X. EXPECTED VALUE is {0x0280}",
           config_reg_val);

  // Modify KP config
  const uint16_t SQ_FIELD_VAL = BOS1901_SQ_SQUARE_WAVE;
  const uint16_t SQ_FIELD_SHIFT = 11;
  const uint16_t SQ_FIELD_MASK_VALUE = 0b1;
  const uint16_t SQ_FIELD_MASK = SQ_FIELD_MASK_VALUE << SQ_FIELD_SHIFT;
  const uint16_t new_kp_value =
      (kp_reg_val & (~SQ_FIELD_MASK)) | (SQ_FIELD_VAL << SQ_FIELD_SHIFT);
  bos1901_device_write_reg(BOS1901_TEST_DEVICE, BOS1901_REG_KP, new_kp_value);

  // Modify PARCAP
  const uint16_t UPI_FIELD_VAL = BOS1901_UPI_ENABLE;
  const uint16_t UPI_FIELD_SHIFT = 11;
  const uint16_t UPI_FIELD_MASK_VALUE = 0b1;
  const uint16_t UPI_FIELD_MASK = UPI_FIELD_MASK_VALUE << UPI_FIELD_SHIFT;

  const uint16_t LMI_FIELD_VAL = BOS1901_LMI_OVERRIDE;
  const uint16_t LMI_FIELD_SHIFT = 10;
  const uint16_t LMI_FIELD_MASK_VALUE = 0b1;
  const uint16_t LMI_FIELD_MASK = LMI_FIELD_MASK_VALUE << LMI_FIELD_SHIFT;

  const uint16_t PARCAP_MASK = ~(UPI_FIELD_MASK | LMI_FIELD_MASK);
  const uint16_t new_parcap_value = (parcap_reg_val & PARCAP_MASK) |
                                    (UPI_FIELD_VAL << UPI_FIELD_SHIFT) |
                                    (LMI_FIELD_VAL << LMI_FIELD_SHIFT);
  bos1901_device_write_reg(BOS1901_TEST_DEVICE, BOS1901_REG_PARCAP,
                           new_parcap_value);

  // Modify config (do not update CONFIG directly!!!)
  const uint16_t PLAY_FIELD_VAL = BOS1901_PLAY_8_KSPS;  // play
  const uint16_t PLAY_FIELD_SHIFT = 0;
  const uint16_t PLAY_FIELD_MASK_VALUE = 0b111;
  const uint16_t PLAY_FIELD_MASK = PLAY_FIELD_MASK_VALUE << PLAY_FIELD_SHIFT;
  const uint16_t new_config_value = (config_reg_val & (~PLAY_FIELD_MASK)) |
                                    (PLAY_FIELD_VAL << PLAY_FIELD_SHIFT);
  bos1901_device_write_reg(BOS1901_TEST_DEVICE, BOS1901_REG_CONFIG,
                           new_config_value);
  BOS1901_TEST_DEVICE->dev_config->play_mode = BOS1901_PLAY_8_KSPS;

  // Verify changes
  uint16_t config_reg_new_val, kp_reg_new_val, parcap_reg_new_val;
  bos1901_device_read_reg(BOS1901_TEST_DEVICE, BOS1901_REG_PARCAP,
                          &parcap_reg_new_val);
  bos1901_device_read_reg(BOS1901_TEST_DEVICE, BOS1901_REG_KP, &kp_reg_new_val);
  bos1901_device_read_reg(BOS1901_TEST_DEVICE, BOS1901_REG_CONFIG,
                          &config_reg_new_val);

  parcap_reg_new_val &= 0xFFF;
  kp_reg_new_val &= 0xFFF;
  config_reg_new_val &= 0xFFF;

  ESP_LOGI(TAG, "After modification, KP result: {0x%04X}, should be: {0x%04X}",
           kp_reg_new_val, new_kp_value);
  ESP_LOGI(TAG,
           "After modification, PARCAP result: {0x%04X}, should be: {0x%04X}",
           parcap_reg_new_val, new_parcap_value);
  ESP_LOGI(TAG,
           "After modification, CONFIG result: {0x%04X}, should be: {0x%04X}",
           config_reg_new_val, new_config_value);
}

static void enable_and_disable_bos1901(TaskHandle_t handle)
{
  ESP_LOGI(TAG, "Enter enable_and_disable_bos1901");

  if (xMutex != NULL)
  {
    // 嘗試獲取互斥鎖
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      bos1901_device_output_enable(BOS1901_TEST_DEVICE, BOS1901_OE_ENABLE);
      ESP_LOGW(TAG,
               "Output enabled! The output value should be: 25 Volt. Start "
               "your measurement. The enable time keep for 10 seconds");
      gpio_set_level(PIN_LED_RED, 1);
      xSemaphoreGive(xMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(10000));

    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
    {
      bos1901_device_output_enable(BOS1901_TEST_DEVICE, BOS1901_OE_DISABLE);
      vTaskDelete(handle);
      xSemaphoreGive(xMutex);
    }
    gpio_set_level(PIN_LED_RED, 0);
    ESP_LOGW(TAG, "Output disabled!");
  }
}

void app_main()
{
  esp_log_level_set("bos1901", ESP_LOG_DEBUG);

  vTaskDelay(pdMS_TO_TICKS(1000));

  init_spi();

  test_bos1901_device();

  configure_hand_bos1901_test();

  xMutex = xSemaphoreCreateMutex();
  if (xMutex == NULL)
  {
    ESP_LOGE(TAG, "Mutex creation failed!");
    return;
  }

  TaskHandle_t fifo_task_handle;

  /* add task here */
  xTaskCreate(fill_fifo_to_bos1901_0_task,  // Task 函數
              "FillFifoTask",               // Task 名稱
              4096,                         // Task Stack 大小
              NULL,                         // Task 函數參數
              5,                            // Task 優先級
              &fifo_task_handle);           // Task handle (可選)

  enable_and_disable_bos1901(fifo_task_handle);

  // // read desired voltage
  // uint16_t output_amp;
  // bos1901_device_read_reg(bos1901_device_0, BOS1901_SUB_REG_OUTPUT_AMP,
  //                         &output_amp);
  // output_amp &= ~(1 << 12);
  // ESP_LOGI(TAG, "Desired voltage should be %.3f", amp_to_volt(output_amp));

  // bos1901_device_deinit(bos1901_device_0);
  // bos1901_device_deinit(bos1901_device_1);
  // bos1901_device_deinit(bos1901_device_2);
  // bos1901_device_deinit(bos1901_device_3);
}
