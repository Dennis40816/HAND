#ifndef _VL53L1X_H_
#define _VL53L1X_H_

#include "vl53l1_error_codes.h"
#include "vl53l1_platform.h"

VL53L1_Error VL53L1X_SetI2CAddress(VL53L1_DEV dev, uint8_t new_address);

#endif