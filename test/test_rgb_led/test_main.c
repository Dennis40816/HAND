#include "led_strip.h"
#include "freertos/task.h"
#include "esp_log.h"

#define BLINK_GPIO_NUM 45

static const char* TAG = "HAND_RGB_LED_TEST";

void app_main()
{
  led_strip_handle_t led_strip;

  /* LED strip initialization with the GPIO and pixels number*/
  led_strip_config_t strip_config = {
      .strip_gpio_num = BLINK_GPIO_NUM,  // The GPIO that connected to the LED
                                         // strip's data line
      .max_leds = 1,                     // The number of LEDs in the strip,
      .led_pixel_format =
          LED_PIXEL_FORMAT_GRB,       // Pixel format of your LED strip
      .led_model = LED_MODEL_WS2812,  // LED strip model
      .flags.invert_out = false,  // whether to invert the output signal (useful
                                  // when your hardware has a level inverter)
  };

  led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
      .rmt_channel = 0,
#else
      .clk_src = RMT_CLK_SRC_DEFAULT,     // different clock source can lead to
                                          // different power consumption
      .resolution_hz = 10 * 1000 * 1000,  // 10MHz
      .flags.with_dma = false,            // whether to enable the DMA feature
#endif
  };

  ESP_ERROR_CHECK(
      led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

  bool led_on_off = false;

  while (1)
  {
    if (led_on_off)
    {
      /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
      for (int i = 0; i < 1; i++)
      {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 32, 64));
      }
      /* Refresh the strip to send data */
      ESP_ERROR_CHECK(led_strip_refresh(led_strip));
      ESP_LOGI(TAG, "LED ON!");
    }
    else
    {
      /* Set all LED off to clear all pixels */
      ESP_ERROR_CHECK(led_strip_clear(led_strip));
      ESP_LOGI(TAG, "LED OFF!");
    }

    led_on_off = !led_on_off;
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}