#ifndef _VL53L1X_H_
#define _VL53L1X_H_

/* include api */
#include "vl53l1_api.h"

#include "vl53l1_error_codes.h"
#include "vl53l1_platform.h"

/* bsp include */
#ifdef LIB_BSP_BOARD
#include "vl53l1_bsp.h"
#endif

VL53L1_Error VL53L1X_SetI2CAddress(VL53L1_DEV dev, uint8_t new_address);

#endif