#pragma once

#include "driver/i2c.h"
#include "vl53l1_error_codes.h"
#include "tca6408a.h"

VL53L1_Error VL53L1_ConfigTca6408a(tca6408a_i2c_t i2c_port);