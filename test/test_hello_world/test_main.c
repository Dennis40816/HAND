#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <unity.h>
#include "driver/gpio.h"

static const char *LOG_TAG = "test_hello_world";

#define LOG_BUFFER_SIZE 1024
static char log_buffer[LOG_BUFFER_SIZE];
static uint32_t log_buffer_index = 0;

static int custom_log_vprintf(const char *fmt, va_list args)
{
  int len = vsnprintf(log_buffer + log_buffer_index,
                      LOG_BUFFER_SIZE - log_buffer_index, fmt, args);
  log_buffer_index += len;
  if (log_buffer_index >= LOG_BUFFER_SIZE)
  {
    log_buffer_index = 0;  // Reset if overflow
  }
  return len;
}

void setUp()
{
  log_buffer_index = 0;
  // Set custom log handler
  esp_log_set_vprintf(custom_log_vprintf);
}

void tearDown()
{
  // Restore default log handler
  esp_log_set_vprintf(vprintf);
}

void init_gpio()
{
  // 配置 GPIO39
  gpio_config_t io_conf_43 = {.intr_type = GPIO_INTR_DISABLE,
                              .mode = GPIO_MODE_OUTPUT,
                              .pin_bit_mask = (1ULL << GPIO_NUM_43),
                              .pull_down_en = GPIO_PULLDOWN_DISABLE,
                              .pull_up_en = GPIO_PULLUP_DISABLE};
  gpio_config(&io_conf_43);

  // 配置 GPIO40
  gpio_config_t io_conf_44 = {.intr_type = GPIO_INTR_DISABLE,
                              .mode = GPIO_MODE_OUTPUT,
                              .pin_bit_mask = (1ULL << GPIO_NUM_44),
                              .pull_down_en = GPIO_PULLDOWN_DISABLE,
                              .pull_up_en = GPIO_PULLUP_DISABLE};
  gpio_config(&io_conf_44);
  gpio_set_level(GPIO_NUM_43, 0); 
  gpio_set_level(GPIO_NUM_44, 0); 
}

void blink_led_txd_task(void *pvParameter)
{
  while (1)
  {
    gpio_set_level(GPIO_NUM_43, 1);         // 打開 LED
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // 延遲 1 秒
    ESP_LOGI(LOG_TAG, "Red");
    gpio_set_level(GPIO_NUM_43, 0);         // 關閉 LED
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // 延遲 1 秒
  }
}

void blink_led_rxd_task(void *pvParameter)
{
  while (1)
  {
    gpio_set_level(GPIO_NUM_44, 0);         // 打開 LED
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // 延遲 1 秒
    ESP_LOGI(LOG_TAG, "White");
    gpio_set_level(GPIO_NUM_44, 1);         // 關閉 LED
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // 延遲 1 秒
  }
}

void debug_log_task(void *pvParameter)
{
  while (1)
  {
    ESP_LOGI("debug_log_task", "Current time is: %lld us",
             esp_timer_get_time());
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void test_log_output()
{
  const char *expected_log = "Hello World, HAND is here for you!";
  ESP_LOGI(LOG_TAG, "%s", expected_log);

  TEST_ASSERT_NOT_NULL(strstr(log_buffer, expected_log));
}

void test_debug_log_task()
{
  ESP_LOGI("test_debug_log_task", "Current time is: %lld us", 1000LL);

  TEST_ASSERT_NOT_NULL(strstr(log_buffer, "Current time is: 1000 us"));
}

void app_main()
{
  UNITY_BEGIN();
  RUN_TEST(test_log_output);
  RUN_TEST(test_debug_log_task);

  /* wait until PC ready */
  vTaskDelay(pdMS_TO_TICKS(1000));

  ESP_LOGI(LOG_TAG, "Hello World, HAND is here for you!");

  /* prepare led */
  init_gpio();

  /* start blinking tasks */
  xTaskCreate(&blink_led_txd_task, "blink_led_txd_task", 2048, NULL, 5, NULL);
  xTaskCreate(&blink_led_rxd_task, "blink_led_rxd_task", 2048, NULL, 6, NULL);
  xTaskCreate(&debug_log_task, "debug_log_task", 2048, NULL, 7, NULL);
  ESP_LOGI(LOG_TAG, "Start blinking tasks! Please check LEDs 5, 6 are blinking.");
  vTaskDelay(pdMS_TO_TICKS(10000));
  UNITY_END();
}
