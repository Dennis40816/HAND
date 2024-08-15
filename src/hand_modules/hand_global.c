#include "hand_global.h"
#include "hand_data/hand_data.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* HAND related */

hand_devices_handle_t hand_global_devs_handle = {0};
hand_task_handle_t hand_global_task_handle = {0};

/* CH101 related */

uint8_t hand_global_ch101_active_dev_num = 0;
uint8_t hand_global_ch101_triggered_dev_num = 0;

/* Ping pong buffer related */

// VL53L1X related
SemaphoreHandle_t hand_global_vl53l1x_ping_pong_mutex = NULL;
hand_ppb_vl53l1x_data_t hand_global_vl53l1x_ping_pong_buffer = {0};

// CH101
SemaphoreHandle_t hand_global_ch101_ping_pong_mutex = NULL;
hand_ppb_ch101_data_t hand_global_ch101_ping_pong_buffer = {0};

/* Queue related */

// VL53L1X
QueueHandle_t hand_global_vl53l1x_data_queue = NULL;

// CH101
QueueHandle_t hand_global_ch101_data_queue = NULL;

/* Event group related */

// VL53L1X related
EventGroupHandle_t hand_global_vl53l1x_event_group = NULL;

// CH101 related
EventGroupHandle_t hand_global_ch101_event_group = NULL;

/* public API */
esp_err_t hand_global_var_init()
{
  esp_err_t ret = ESP_OK;

  /* init mutex */
  hand_global_vl53l1x_ping_pong_mutex = xSemaphoreCreateMutex();
  hand_global_ch101_ping_pong_mutex = xSemaphoreCreateMutex();

  /* init queue */
  hand_global_vl53l1x_data_queue = xQueueCreate(
      HAND_SIZE_QUEUE_VL53L1X, sizeof(hand_vl53l1x_data_element_t));
  hand_global_ch101_data_queue =
      xQueueCreate(HAND_SIZE_QUEUE_CH101, sizeof(hand_chx01_data_element_t));

  /* init event group */
  hand_global_vl53l1x_event_group = xEventGroupCreate();
  hand_global_ch101_event_group = xEventGroupCreate();

  /* TODO: init device handle */

  return ret;
}

/* TODO: Callback related, move to hand_cb.c? */
void IRAM_ATTR hand_cb_vl53l1x_sensed(void* arg)
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