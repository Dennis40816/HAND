/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * This file is part of VL53L1 Platform
 *
 * Copyright (c) 2016, STMicroelectronics - All Rights Reserved
 *
 * License terms: BSD 3-clause "New" or "Revised" License.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "rom/ets_sys.h"
#include "esp_timer.h"

/* platform include */
#include "vl53l1_platform.h"
#include "vl53l1_platform_log.h"

/* core include */
#include "vl53l1_api.h"

/* weak link declaration */

#define WRITE_BIT         I2C_MASTER_WRITE
#define READ_BIT          I2C_MASTER_READ
#define ACK_CHECK_EN      0x1
#define ACK_CHECK_DISABLE 0x0
#define ACK_VAL           0x0
#define NACK_VAL          0x1

/* const static variables */

static enum static_constexpr_alternative {
  INDEX_LEN = 2,
  BYTE_LEN = 1,
  WORD_LEN = 2,
  DWORD_LEN = 4
};
// static const size_t INDEX_LEN = 2;
// static const size_t BYTE_LEN = 1;
// static const size_t WORD_LEN = 2;
// static const size_t DWORD_LEN = 4;

static esp_err_t _I2CWrite(VL53L1_DEV Dev, uint8_t *buf, uint32_t len)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (Dev->I2cDevAddr | WRITE_BIT), ACK_CHECK_EN);
  i2c_master_write(cmd, buf, len, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(Dev->I2cHandle, cmd, pdMS_TO_TICKS(1000));
  ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    return -1;
  }
  return 0;
}

static esp_err_t _I2CRead(VL53L1_DEV Dev, uint8_t *buf, uint32_t len)
{
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (Dev->I2cDevAddr | READ_BIT), ACK_CHECK_EN);
  if (len > 1)
  {
    i2c_master_read(cmd, buf, len - 1, ACK_VAL);
  }
  i2c_master_read_byte(cmd, buf + len - 1, NACK_VAL);
  i2c_master_stop(cmd);
  esp_err_t ret =
      i2c_master_cmd_begin(Dev->I2cHandle, cmd, pdMS_TO_TICKS(1000));
  ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
  i2c_cmd_link_delete(cmd);
  return ret;
}

static void VL53L1_GetI2cBus(void) {}

static void VL53L1_PutI2cBus(void) {}

__attribute__((weak)) VL53L1_Error VL53L1_WriteMulti(VL53L1_DEV Dev,
                                                     uint16_t index,
                                                     uint8_t *pdata,
                                                     uint32_t count)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t index_buf[INDEX_LEN] = {index >> 8, index & 0xFF};

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (Dev->I2cDevAddr | WRITE_BIT), ACK_CHECK_EN);
  // TODO: this will fail when using <i2c_master.h>
  // write internal address to VL53L1
  i2c_master_write(cmd, index_buf, INDEX_LEN, ACK_CHECK_EN);
  // write data
  i2c_master_write(cmd, pdata, count, ACK_CHECK_EN);
  i2c_master_stop(cmd);

  VL53L1_GetI2cBus();

  esp_err_t ret =
      i2c_master_cmd_begin(Dev->I2cHandle, cmd, pdMS_TO_TICKS(1000));
  ESP_ERROR_CHECK_WITHOUT_ABORT(ret);
  i2c_cmd_link_delete(cmd);

  VL53L1_PutI2cBus();

  if (ret != ESP_OK)
  {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

  return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
__attribute__((weak)) VL53L1_Error VL53L1_ReadMulti(VL53L1_DEV Dev,
                                                    uint16_t index,
                                                    uint8_t *pdata,
                                                    uint32_t count)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t index_buf[INDEX_LEN] = {index >> 8, index & 0xFF};

  VL53L1_GetI2cBus();

  esp_err_t ret = _I2CWrite(Dev, index_buf, INDEX_LEN);
  if (ret != ESP_OK)
  {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
    goto done;
  }
  ret = _I2CRead(Dev, pdata, count);
  if (ret != ESP_OK)
  {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

done:
  VL53L1_PutI2cBus();
  return Status;
}

__attribute__((weak)) VL53L1_Error VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index,
                                                 uint8_t data)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t buf[INDEX_LEN + BYTE_LEN] = {
      [0] = index >> 8, [1] = index & 0xFF, [2] = data};

  VL53L1_GetI2cBus();

  esp_err_t ret = _I2CWrite(Dev, buf, INDEX_LEN + BYTE_LEN);
  if (ret != ESP_OK)
  {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

  VL53L1_PutI2cBus();
  return Status;
}

__attribute__((weak)) VL53L1_Error VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index,
                                                 uint16_t data)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t buf[INDEX_LEN + WORD_LEN] = {[0] = index >> 8,
                                       [1] = index & 0xFF,
                                       [2] = data >> 8,
                                       [3] = data & 0x00FF};

  VL53L1_GetI2cBus();

  esp_err_t ret = _I2CWrite(Dev, buf, INDEX_LEN + WORD_LEN);
  if (ret != ESP_OK)
  {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

  VL53L1_PutI2cBus();
  return Status;
}

__attribute__((weak)) VL53L1_Error VL53L1_WrDWord(VL53L1_DEV Dev,
                                                  uint16_t index, uint32_t data)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;

  uint8_t buf[INDEX_LEN + DWORD_LEN] = {
      [0] = index >> 8,          [1] = index & 0xFF,
      [2] = (data >> 24) & 0xFF, [3] = (data >> 16) & 0xFF,
      [4] = (data >> 8) & 0xFF,  [5] = (data >> 0) & 0xFF};

  VL53L1_GetI2cBus();

  esp_err_t ret = _I2CWrite(Dev, buf, INDEX_LEN + DWORD_LEN);
  if (ret != ESP_OK)
  {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

  VL53L1_PutI2cBus();
  return Status;
}

__attribute__((weak)) VL53L1_Error VL53L1_UpdateByte(VL53L1_DEV Dev,
                                                     uint16_t index,
                                                     uint8_t AndData,
                                                     uint8_t OrData)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t data;

  Status = VL53L1_RdByte(Dev, index, &data);
  if (Status)
  {
    goto done;
  }
  data = (data & AndData) | OrData;
  Status = VL53L1_WrByte(Dev, index, data);
done:
  return Status;
}

__attribute__((weak)) VL53L1_Error VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index,
                                                 uint8_t *data)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t index_buf[INDEX_LEN] = {[0] = index >> 8, [1] = index & 0xFF};

  VL53L1_GetI2cBus();

  esp_err_t ret = _I2CWrite(Dev, index_buf, INDEX_LEN);
  if (ret != ESP_OK)
  {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
    goto done;
  }
  ret = _I2CRead(Dev, data, 1);
  if (ret != ESP_OK)
  {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
  }

done:
  VL53L1_PutI2cBus();
  return Status;
}

__attribute__((weak)) VL53L1_Error VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index,
                                                 uint16_t *data)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t index_buf[INDEX_LEN] = {[0] = index >> 8, [1] = index & 0xFF};
  uint8_t rx_buf[WORD_LEN] = {0};

  VL53L1_GetI2cBus();

  esp_err_t ret = _I2CWrite(Dev, index_buf, INDEX_LEN);

  if (ret != ESP_OK)
  {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
    goto done;
  }
  ret = _I2CRead(Dev, rx_buf, WORD_LEN);
  if (ret != ESP_OK)
  {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
    goto done;
  }

  *data = ((uint16_t)rx_buf[0] << 8) + (uint16_t)rx_buf[1];

done:
  VL53L1_PutI2cBus();
  return Status;
}

__attribute__((weak)) VL53L1_Error VL53L1_RdDWord(VL53L1_DEV Dev,
                                                  uint16_t index,
                                                  uint32_t *data)
{
  VL53L1_Error Status = VL53L1_ERROR_NONE;
  uint8_t index_buf[INDEX_LEN] = {[0] = index >> 8, [1] = index & 0xFF};
  uint8_t rx_buf[DWORD_LEN] = {0};

  VL53L1_GetI2cBus();

  esp_err_t ret = _I2CWrite(Dev, index_buf, INDEX_LEN);
  if (ret != ESP_OK)
  {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
    goto done;
  }
  ret = _I2CRead(Dev, rx_buf, DWORD_LEN);
  if (ret != ESP_OK)
  {
    Status = VL53L1_ERROR_CONTROL_INTERFACE;
    goto done;
  }

  *data = ((uint32_t)rx_buf[0] << 24) + ((uint32_t)rx_buf[1] << 16) +
          ((uint32_t)rx_buf[2] << 8) + (uint32_t)rx_buf[3];

done:
  VL53L1_PutI2cBus();
  return Status;
}

__attribute__((weak)) VL53L1_Error VL53L1_GetTickCount(uint32_t *ptick_count_ms)
{
  VL53L1_Error status = VL53L1_ERROR_NONE;
  int64_t sys_us = esp_timer_get_time();
  *ptick_count_ms = sys_us / 1000;

  return status;
}

/* unused */
__attribute__((weak)) VL53L1_Error
VL53L1_GetTimerFrequency(int32_t *ptimer_freq_hz)
{
  // system timer
  *ptimer_freq_hz = 1000000;

  return VL53L1_ERROR_NONE;
}

__attribute__((weak)) VL53L1_Error VL53L1_WaitMs(VL53L1_Dev_t *pdev,
                                                 int32_t wait_ms)
{
  (void)pdev;
  vTaskDelay(wait_ms / portTICK_PERIOD_MS);
  return VL53L1_ERROR_NONE;
}

__attribute__((weak)) VL53L1_Error VL53L1_WaitUs(VL53L1_Dev_t *pdev,
                                                 int32_t wait_us)
{
  (void)pdev;
  /* when wait_us is -1 => block */
  ets_delay_us(wait_us);
  return VL53L1_ERROR_NONE;
}

/* TODO */
__attribute__((weak)) VL53L1_Error
VL53L1_WaitValueMaskEx(VL53L1_Dev_t *pdev, uint32_t timeout_ms, uint16_t index,
                       uint8_t value, uint8_t mask, uint32_t poll_delay_ms)
{
  /*
   * Platform implementation of WaitValueMaskEx V2WReg script command
   *
   * WaitValueMaskEx(
   *          duration_ms,
   *          index,
   *          value,
   *          mask,
   *          poll_delay_ms);
   */

  VL53L1_Error status = VL53L1_ERROR_NONE;
  uint32_t start_time_ms = 0;
  uint32_t current_time_ms = 0;
  uint32_t polling_time_ms = 0;
  uint8_t byte_value = 0;
  uint8_t found = 0;
  char register_name[VL53L1_MAX_STRING_LENGTH];

  /* look up register name */
#ifdef PAL_EXTENDED
  VL53L1_get_register_name(index, register_name);
#else
  VL53L1_COPYSTRING(register_name, "");
#endif

  /* calculate time limit in absolute time */

  VL53L1_GetTickCount(&start_time_ms);

  /* wait until value is found, timeout reached on error occurred */

  while ((status == VL53L1_ERROR_NONE) && (polling_time_ms < timeout_ms) &&
         (found == 0))
  {
    if (status == VL53L1_ERROR_NONE)
    {
      status = VL53L1_RdByte(pdev, index, &byte_value);
    }

    if ((byte_value & mask) == value)
    {
      found = 1;
    }

    if (status == VL53L1_ERROR_NONE && found == 0 && poll_delay_ms > 0)
    {
      status = VL53L1_WaitMs(pdev, poll_delay_ms);
    }

    /* Update polling time (Compare difference rather than absolute to
    negate 32bit wrap around issue) */
    VL53L1_GetTickCount(&current_time_ms);
    polling_time_ms = current_time_ms - start_time_ms;
  }

  if (found == 0 && status == VL53L1_ERROR_NONE)
  {
    status = VL53L1_ERROR_TIME_OUT;
  }

  return status;
}

/* gpio */
__attribute__((weak)) VL53L1_Error VL53L1_GpioSetMode(uint8_t pin, uint8_t mode)
{
  VL53L1_Error status = VL53L1_ERROR_NONE;
  esp_err_t ret = gpio_set_direction(pin, mode);

  if (ret != ESP_OK)
  {
    status = VL53L1_ERROR_PLATFORM_GPIO_SET_MODE;
  }

  return status;
}

__attribute__((weak)) VL53L1_Error VL53L1_GpioSetValue(uint8_t pin,
                                                       uint8_t value)
{
  VL53L1_Error status = VL53L1_ERROR_NONE;
  esp_err_t ret = gpio_set_level(pin, value);

  if (ret != ESP_OK)
  {
    status = VL53L1_ERROR_PLATFORM_GPIO_SET_VALUE;
  }

  return status;
}

__attribute__((weak)) VL53L1_Error VL53L1_GpioGetValue(uint8_t pin,
                                                       uint8_t *pvalue)
{
  VL53L1_Error status = VL53L1_ERROR_NONE;
  *pvalue = gpio_get_level(pin);

  return status;
}

__attribute__((weak)) VL53L1_Error VL53L1_GpioXshutdown(uint8_t pin,
                                                        uint8_t value)
{
  return VL53L1_GpioSetValue(pin, value);
}

__attribute__((weak)) VL53L1_Error VL53L1_GpioInterruptEnable(uint8_t pin,
                                                              gpio_isr_cb_fn fn,
                                                              uint8_t edge_type)
{
  esp_err_t ret;
  VL53L1_Error status = VL53L1_ERROR_NONE;

  ret = gpio_set_intr_type(pin, edge_type);
  if (ret != ESP_OK)
  {
    status = VL53L1_ERROR_PLATFORM_GPIO_SET_INTR;
    return status;
  }

  ret = gpio_intr_enable(pin);
  if (ret != ESP_OK)
  {
    status = VL53L1_ERROR_PLATFORM_GPIO_EN_INTR;
    return status;
  }

  /* add callback with no args */
  ret = gpio_isr_handler_add(pin, fn, NULL);
  if (ret != ESP_OK)
  {
    status = VL53L1_ERROR_PLATFORM_GPIO_REGISTER_ISR;
  }
  return status;
}

__attribute__((weak)) VL53L1_Error VL53L1_GpioInterruptDisable(uint8_t pin)
{
  esp_err_t ret;
  VL53L1_Error status = VL53L1_ERROR_NONE;

  ret = gpio_intr_disable(pin);
  if (ret != ESP_OK)
  {
    status = VL53L1_ERROR_PLATFORM_GPIO_EN_INTR;
    return status;
  }

  ret = gpio_isr_handler_remove(pin);
  if (ret != ESP_OK)
  {
    status = VL53L1_ERROR_PLATFORM_GPIO_EN_INTR;
  }

  return status;
}
