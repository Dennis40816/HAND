#include "vl53l1_bsp.h"
#include "vl53l1_platform_log.h"

#include "tca6408a.h"

/**
 * For HAND
 * VL53L1X_1 XSHUT: TCA6408A (0x20) P3
 * VL53L1X_2 XSHUT: TCA6408A (0x20) P5
 */

static tca6408a_dev_t tca6408a_other = {.address = 0x20};
static const char* TAG = "VL53L1X";

/* Call this before using VL53L1_GpioXshutdown and make sure i2c port is already
 * initialized outside */
VL53L1_Error VL53L1_ConfigTca6408a(tca6408a_i2c_t i2c_port)
{
  tca6408a_other.i2c_bus = i2c_port;

  /* config P3 and P5 as output, default disable VL53L1 (low) */
  tca6408a_set_pin_output_mode(&tca6408a_other, 3);
  tca6408a_set_pin_output_mode(&tca6408a_other, 5);

  tca6408a_set_pin_low(&tca6408a_other, 3);
  tca6408a_set_pin_low(&tca6408a_other, 5);

  return VL53L1_ERROR_NONE;
}

/* Only pin == 1 or 2 is valid */
VL53L1_Error VL53L1_GpioXshutdown(uint8_t pin, uint8_t value)
{
  /* make sure i2c (SCL: IO 0, SDA: IO 1) is initialized */
  tca6408a_err_t ret = 0;

  if (pin != 1 && pin != 2)
  {
    return VL53L1_ERROR_PLATFORM_XSHUT_INDEX;
  }

  // VL53L1X_1 XSHUT
  if (pin == 1)
  {
    if (value)
    {
      ret = tca6408a_set_pin_high(&tca6408a_other, 3);
    }
    else
    {
      ret = tca6408a_set_pin_low(&tca6408a_other, 3);
    }
  }
  // VL53L1X_2 XSHUT
  else
  {
    if (value)
    {
      ret = tca6408a_set_pin_high(&tca6408a_other, 5);
    }
    else
    {
      ret = tca6408a_set_pin_low(&tca6408a_other, 5);
    }
  }

  VL_LOGD(TAG, "Set VL53L1X_{%d} XSHUT (tca6408a_other P{%d}) to {%s}", pin,
          ((pin == 1) ? 3 : 5), (value) ? "high" : "low");

  if (ret != 0)
  {
    return VL53L1_ERROR_CONTROL_INTERFACE;
  }

  return VL53L1_ERROR_NONE;
}