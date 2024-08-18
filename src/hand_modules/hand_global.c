#include "hand_global.h"
#include "hand_data/hand_data.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const char *TAG = "HAND_GLOBAL";

/* TODO: should we use volatile? */

/* HAND related */

hand_devices_handle_t hand_global_devs_handle = {0};
hand_task_handle_t hand_global_task_handle = {0};

/* CH101 related */

volatile uint8_t hand_global_ch101_active_dev_num = 0;
volatile uint8_t hand_global_ch101_triggered_dev_num = 0;

/* Ping pong buffer related */

// VL53L1X related
volatile SemaphoreHandle_t hand_global_vl53l1x_ping_pong_mutex = NULL;
volatile hand_ppb_vl53l1x_data_t hand_global_vl53l1x_ping_pong_buffer = {0};

// CH101
volatile SemaphoreHandle_t hand_global_ch101_ping_pong_mutex = NULL;
volatile hand_ppb_ch101_data_t hand_global_ch101_ping_pong_buffer = {0};

/* Queue related */

// VL53L1X
volatile QueueHandle_t hand_global_vl53l1x_data_queue = NULL;

// CH101
volatile QueueHandle_t hand_global_ch101_data_queue = NULL;

/* Event group related */

// VL53L1X related
volatile EventGroupHandle_t hand_global_vl53l1x_event_group = NULL;

// CH101 related
volatile EventGroupHandle_t hand_global_ch101_event_group = NULL;

// HAND system related
volatile EventGroupHandle_t hand_global_system_event_group = NULL;

/* public API */
esp_err_t hand_global_var_init()
{
  esp_err_t ret = ESP_OK;
  ESP_LOGI(TAG, "hand_global_var_init starts");

  /* init mutex */
  hand_global_vl53l1x_ping_pong_mutex = xSemaphoreCreateMutex();
  hand_global_ch101_ping_pong_mutex = xSemaphoreCreateMutex();

  /* init queue */
  hand_global_vl53l1x_data_queue = xQueueCreate(
      HAND_SIZE_QUEUE_VL53L1X, sizeof(hand_vl53l1x_data_element_t));
  hand_global_ch101_data_queue = xQueueCreate(
      HAND_SIZE_QUEUE_CH101, sizeof(hand_chx01_group_data_element_t));

  /* init event group */
  hand_global_vl53l1x_event_group = xEventGroupCreate();
  hand_global_ch101_event_group = xEventGroupCreate();
  hand_global_system_event_group = xEventGroupCreate();

  // make LED is controlled by `hand_task_alive`
  xEventGroupSetBits(hand_global_system_event_group,
                     HAND_EG_SYSTEM_LED_CONTROL_BY_ALIVE);

  /* TODO: init device handle */

  return ret;
}

/* TODO: Callback related, move to hand_cb.c? */
void hand_cb_vl53l1x_sensed(void *arg)
{
  int pin_num = (int)arg;

  if (pin_num == HAND_PIN_INT_VL53L1X_1_GPIO1)
  {
    xEventGroupSetBitsFromISR(hand_global_vl53l1x_event_group,
                              HAND_EG_VL53L1X_1_DATA_READY_BIT, NULL);
  }
  else if (pin_num == HAND_PIN_INT_VL53L1X_2_GPIO1)
  {
    xEventGroupSetBitsFromISR(hand_global_vl53l1x_event_group,
                              HAND_EG_VL53L1X_2_DATA_READY_BIT, NULL);
  }
}

void hand_cb_ch101_sensed(ch_group_t *grp_ptr, uint8_t dev_num)
{
  ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
  xEventGroupSetBitsFromISR(hand_global_ch101_event_group, (1 << dev_num),
                            NULL);

  /* TODO: This would be block? */
  EventBits_t event_bits =
      xEventGroupGetBitsFromISR(hand_global_ch101_event_group);

  /* starts group measurement only when all devices got triggered */
  if (event_bits == hand_global_ch101_active_dev_num)
  {
    /* clear data ready event for all active devices */
    xEventGroupClearBitsFromISR(hand_global_ch101_event_group,
                                hand_global_ch101_active_dev_num);

    xEventGroupSetBitsFromISR(hand_global_ch101_event_group,
                              HAND_EG_CH101_ALL_ACTIVE_DEV_DATA_READY_BIT,
                              NULL);

    /* Disable interrupt unless in free-running mode
     *   It will automatically be re-enabled by the next ch_group_trigger()
     */
    if (ch_get_mode(dev_ptr) == CH_MODE_FREERUN)
    {
      /* Clear IO here because we are use TXB0104 */
      chbsp_group_set_io_dir_out(grp_ptr);
      chbsp_group_io_clear(grp_ptr);
      chbsp_group_set_io_dir_in(grp_ptr);  // set INT line as input
      chbsp_delay_us(CHDRV_TRIGGER_PULSE_US);
      chbsp_group_io_interrupt_enable(grp_ptr);
    }
    else
    {
      chbsp_group_io_interrupt_disable(grp_ptr);
    }
  }
}

void hand_cb_ch101_io_completed(ch_group_t *grp_ptr)
{
  xEventGroupSetBitsFromISR(hand_global_ch101_event_group,
                            HAND_EG_CH101_IQ_DATA_READY, NULL);
}

void hand_cb_ch101_periodic_timer(void)
{
  if (hand_global_ch101_triggered_dev_num > 0)
  {
    /* Note: resume interrupt */
    ch_group_trigger(&(hand_global_devs_handle.ch101_group));
  }
}