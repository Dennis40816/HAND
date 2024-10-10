#include "vl53l1x.h"

VL53L1_Error VL53L1X_SetI2CAddress(VL53L1_DEV dev, uint8_t new_address)
{
  VL53L1_Error status = 0;
  status =
      VL53L1_WrByte(dev, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_address >> 1);
  dev->I2cDevAddr = new_address;
  return status;
}