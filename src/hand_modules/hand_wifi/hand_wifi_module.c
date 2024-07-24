/*
 * Copyright (c) 2024 Dennis Liu, dennis48161025@gmail.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "hand_wifi_module.h"

#include "string.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi_default.h"
#include "esp_log.h"

/* Private variables */
static const char* TAG = "HAND_WIFI_MODULE";

// internal state
static esp_event_handler_instance_t _event_instance_wifi = NULL;
static esp_event_handler_instance_t _event_instance_ip = NULL;
static esp_event_handler_t _custom_event_handler = NULL;
static uint8_t _retry_number = 0;
static bool _create_event_group_internally = false;

// default config instance
static const hand_wifi_config_t _default_wifi_config = {
    .ssid_len = sizeof(HAND_WIFI_MODULE_DEFAULT_SSID),
    .password_len = sizeof(HAND_WIFI_MODULE_DEFAULT_PASSWORD),
    .ssid = HAND_WIFI_MODULE_DEFAULT_SSID,
    .password = HAND_WIFI_MODULE_DEFAULT_PASSWORD};

// The instance that really take effect
static hand_wifi_t _hand_wifi_internal;
static hand_wifi_handle_t _hand_wifi_internal_handle = &_hand_wifi_internal;

/* Private functions */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
  /* WIFI EVENT */
  if (event_base == WIFI_EVENT)
  {
    switch (event_id)
    {
      /* WIFI_EVENT_STA_START START */
      case WIFI_EVENT_STA_START:
        esp_wifi_connect();
        break;
        /* WIFI_EVENT_STA_START END */

      case WIFI_EVENT_STA_CONNECTED:
        /* reset retry number */
        _retry_number = 0;

        /* wifi event group set bit here */
        xEventGroupSetBits(_hand_wifi_internal_handle->wifi_event_group,
                           HAND_WIFI_CONNECTED_BIT);
        break;

      /* WIFI_EVENT_STA_DISCONNECTED START */
      case WIFI_EVENT_STA_DISCONNECTED:
      {
        if (_retry_number < _hand_wifi_internal_handle->max_retry)
        {
          esp_wifi_connect();
          ++_retry_number;
          ESP_LOGW(TAG, "Retry {%d} time to connect to the AP", _retry_number);
        }
        else
        {
          /* wifi event group set bit here */
          xEventGroupSetBits(_hand_wifi_internal_handle->wifi_event_group,
                             HAND_WIFI_CONNECT_FAIL_BIT);
          ESP_LOGE(TAG,
                   "Try to connect to the AP failed. Max retry number: "
                   "{%d} achieved!",
                   _hand_wifi_internal_handle->max_retry);

          /* reset retry number */
          _retry_number = 0;
        }
        break;
      }
      /* WIFI_EVENT_STA_DISCONNECTED END */

      /* DEFAULT START */
      default:
        ESP_LOGW(TAG, "Wi-Fi event id: {%ld} not implemented!", event_id);
        break;

        /* DEFAULT END */
    }
  }
  else if (event_base == IP_EVENT)
  {
    switch (event_id)
    {
      case IP_EVENT_STA_GOT_IP:
      {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR "\n", IP2STR(&event->ip_info.ip));
      }
    }
  }

  if (_custom_event_handler != NULL)
  {
    _custom_event_handler(arg, event_base, event_id, event_data);
  }
}

static esp_err_t hand_wifi_module_validate_handle(hand_wifi_handle_t handle)
{
  ESP_LOGI(TAG, "Going to validate hand_wifi_handle parameters...");

  /* handle not NULL */
  if (handle == NULL)
  {
    ESP_LOGE(TAG, "INVALID ARG: [handle] is {NULL}");
    return ESP_ERR_INVALID_ARG;
  }

  /* max_retry: no limitation */
  ESP_LOGI(TAG, "[max_retry]: {%d}", handle->max_retry);

  /* interface: no eqal or larger than WIFI_IF_MAX */
  if (handle->interface >= WIFI_IF_MAX)
  {
    ESP_LOGE(TAG,
             "INVALID ARG: [handle->interface] is {%d}. Larger than "
             "'WIFI_IF_MAX': {%d}",
             handle->interface, WIFI_IF_MAX);
    return ESP_ERR_INVALID_ARG;
  }

  /* mode: no equal or larger than WIFI_MODE_MAX */
  if (handle->mode >= WIFI_MODE_MAX)
  {
    ESP_LOGE(TAG,
             "INVALID ARG: [handle->mode] is {%d}. Larger than "
             "'WIFI_MODE_MAX': {%d}",
             handle->mode, WIFI_MODE_MAX);
    return ESP_ERR_INVALID_ARG;
  }

  /* wifi_event_group: no limitation */

  /* config: len check */
  if (handle->config.ssid_len > HAND_WIFI_MODULE_SSID_MAX_LEN)
  {
    ESP_LOGE(TAG,
             "INVALID ARG: [handle->config.ssid_len] is {%d}. Larger than "
             "'HAND_WIFI_MODULE_SSID_MAX_LEN': {%d}",
             handle->config.ssid_len, HAND_WIFI_MODULE_SSID_MAX_LEN);
    return ESP_ERR_INVALID_ARG;
  }
  if (handle->config.password_len > HAND_WIFI_MODULE_PASSWORD_MAX_LEN)
  {
    ESP_LOGE(TAG,
             "INVALID ARG: [handle->config.password_len] is {%d}. Larger than "
             "'HAND_WIFI_MODULE_PASSWORD_MAX_LEN': {%d}",
             handle->config.password_len, HAND_WIFI_MODULE_PASSWORD_MAX_LEN);
    return ESP_ERR_INVALID_ARG;
  }

  /* user: no limitation */

  /* validation end */
  return ESP_OK;
}

/* Public functions */

esp_err_t hand_wifi_module_mount(void* parameter)
{
  /* TODO */
  ESP_ERROR_CHECK(nvs_flash_init());
  return ESP_OK;
}

esp_err_t hand_wifi_module_init(hand_wifi_handle_t handle,
                                bool wait_util_connect)
{
  /* valid check */
  esp_err_t ret = hand_wifi_module_validate_handle(handle);

  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Validate failed! Abort init");
    ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
    return ret;
  }

  /* modify event group create internally state */
  if (handle->wifi_event_group == NULL)
  {
    handle->wifi_event_group = xEventGroupCreate();
    _create_event_group_internally = true;
  }

  /* start to build */
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  ESP_LOGI(TAG, "ESP netif create, mode: {%d} ", handle->mode);

  switch (handle->mode)
  {
    case WIFI_MODE_STA:
      esp_netif_create_default_wifi_sta();
      break;

    case WIFI_MODE_AP:
      esp_netif_create_default_wifi_ap();
      break;

    case WIFI_MODE_APSTA:
      esp_netif_create_default_wifi_ap();
      esp_netif_create_default_wifi_sta();
      break;

    case WIFI_MODE_NAN:
      /* FIXME: undefined reference??? */
      // esp_netif_create_default_wifi_nan();
      break;

    default:
      ESP_LOGE(TAG, "Wi-Fi mode {%d} error!", handle->mode);
      return ESP_ERR_INVALID_ARG;
  }

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  wifi_config_t wifi_config = {0};

  ESP_LOGD(TAG, "Wi-Fi information:\nSSID: {%s}\nPASSWORD: {%s}",
           handle->config.ssid, handle->config.password);

  memcpy(wifi_config.sta.ssid, handle->config.ssid, handle->config.ssid_len);
  memcpy(wifi_config.sta.password, handle->config.password,
         handle->config.password_len);

  /* set mode and interface */
  ESP_ERROR_CHECK(esp_wifi_set_mode(handle->mode));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

  /* copy information to internal instance */
  memcpy(&_hand_wifi_internal, handle, sizeof(hand_wifi_t));

  /* register wifi event handler */
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL,
      &_event_instance_wifi));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL,
      &_event_instance_ip));

  /* start wifi */
  ESP_ERROR_CHECK(esp_wifi_start());

  /* TODO: wait util connected */
  if (wait_util_connect)
  {
    ESP_LOGI(
        TAG,
        "Block: wait_util_connect is set, block util connection is created");

    /* Note: will not clear set bit when exit */
    EventBits_t bits = xEventGroupWaitBits(
        _hand_wifi_internal_handle->wifi_event_group,
        HAND_WIFI_CONNECTED_BIT | HAND_WIFI_CONNECT_FAIL_BIT, pdFALSE, pdFALSE,
        portMAX_DELAY);

    if (bits & HAND_WIFI_CONNECTED_BIT)
    {
      ESP_LOGI(TAG, "Wi-Fi already connects");
    }
    else if (bits & HAND_WIFI_CONNECT_FAIL_BIT)
    {
      ESP_LOGE(TAG, "Failed to connect to SSID:%s, password:%s",
               _hand_wifi_internal_handle->config.ssid,
               _hand_wifi_internal_handle->config.password);
      return ESP_ERR_WIFI_CONN;
    }
  }

  return ESP_OK;
}

esp_err_t hand_wifi_module_deinit(void* parameter)
{
  ESP_LOGI(TAG, "Deinit starts");

  ESP_LOGI(TAG, "Try to disconnect the wifi");

  /* call disconnect first */
  hand_wifi_module_disconnect();

  /* unregister event handler instance  */
  ESP_LOGI(TAG, "Try to unregister event handlers");

  if (_event_instance_wifi != NULL)
  {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_instance_unregister(
        WIFI_EVENT, ESP_EVENT_ANY_ID, _event_instance_wifi));
  }

  if (_event_instance_ip != NULL)
  {
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_handler_instance_unregister(
        IP_EVENT, IP_EVENT_STA_GOT_IP, _event_instance_ip));
  }

  /* delete event group if internally allocate */
  if (_create_event_group_internally)
  {
    if (_hand_wifi_internal_handle->wifi_event_group == NULL)
    {
      ESP_LOGE(TAG, "Error! wifi_event_group is NULL when cleaning");
    }
    else
    {
      ESP_LOGI(TAG, "The wifi event group is allocate by module, deleting...");
      vEventGroupDelete(_hand_wifi_internal_handle->wifi_event_group);
    }

    /* reset the state */
    _create_event_group_internally = false;
  }

  /* try to stop wifi block */
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_stop());

  ESP_LOGI(TAG, "Deinit finished");

  return ESP_OK;
}

esp_err_t hand_wifi_module_connect()
{
  wifi_config_t wifi_config;

  /* len check */
  if (_hand_wifi_internal_handle->config.ssid_len >
      HAND_WIFI_MODULE_SSID_MAX_LEN)
  {
    ESP_LOGE(TAG, "Invalid length of SSID: {%d}",
             _hand_wifi_internal_handle->config.ssid_len);
    return ESP_ERR_INVALID_ARG;
  }

  if (_hand_wifi_internal_handle->config.password_len >
      HAND_WIFI_MODULE_PASSWORD_MAX_LEN)
  {
    ESP_LOGE(TAG, "Invalid length of PASSWORD: {%d}",
             _hand_wifi_internal_handle->config.password_len);
    return ESP_ERR_INVALID_ARG;
  }

  memcpy(wifi_config.sta.ssid, _hand_wifi_internal_handle->config.ssid,
         _hand_wifi_internal_handle->config.ssid_len);
  memcpy(wifi_config.sta.password, _hand_wifi_internal_handle->config.password,
         _hand_wifi_internal_handle->config.password_len);

  /* TODO: assume the last element always be '\0', need check */

  ESP_ERROR_CHECK(
      esp_wifi_set_config(_hand_wifi_internal_handle->interface, &wifi_config));

  ESP_ERROR_CHECK(esp_wifi_connect());
  return ESP_OK;
}

esp_err_t hand_wifi_module_disconnect()
{
  esp_err_t ret = esp_wifi_disconnect();
  ESP_ERROR_CHECK_WITHOUT_ABORT(ret);

  return ret;
}

esp_err_t hand_wifi_module_update_handler(esp_event_handler_t event_handler)
{
  _custom_event_handler = event_handler;
  return ESP_OK;
}

EventGroupHandle_t hand_wifi_module_get_event_group_handle()
{
  return _hand_wifi_internal_handle->wifi_event_group;
}

esp_err_t hand_wifi_module_get_default_handle(hand_wifi_handle_t handle)
{
  handle->max_retry = HAND_WIFI_MODULE_DEFAULT_MAX_RETRY;
  handle->interface = HAND_WIFI_MODULE_DEFAULT_INTERFACE;
  handle->mode = HAND_WIFI_MODULE_DEFAULT_MODE;
  handle->wifi_event_group = NULL;
  handle->user = NULL;

  memcpy(&handle->config, &_default_wifi_config, sizeof(hand_wifi_config_t));

  return ESP_OK;
}