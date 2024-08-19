#pragma once

/* espidf related */
#include "driver/i2c.h"

/* VL53L1 related */
#include "vl53l1_platform.h"

/* other middle device related */
#include "tca6408a.h"

VL53L1_Error VL53L1_ConfigTca6408a(tca6408a_i2c_t i2c_port);