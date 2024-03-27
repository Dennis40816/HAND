#ifndef DEVICE_CONFIG_H_
#define DEVICE_CONFIG_H_

#include "hand_app_config.h"

/**
 * @brief This file includes all pin configs for every devices
 * 
 */

/*========================= CH101 related ===========================*/
#include "user_chx01_config.h"

/*========================= TCA6408A related ===========================*/
#ifdef HAND_BUILD_TARGET_PORTING_CH101

#define TCA6408A_INT_PIN 

#endif

#endif