#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

/**
 * @brief This file aims to
 *
 */

/*========================= Build Type ===========================*/

/* USER MODIFY: Board Operation Mode Begin */
#define HAND_DEBUG
#ifndef HAND_DEBUG
#define HAND_RELEASE
#endif
/* USER MODIFY:Board Operation Mode End */

#ifdef HAND_DEBUG
#define CHDRV_DEBUG
#endif

/*========================= Build Target ===========================*/

/* USER MODIFY: Test Options Begin */
/**
 * @brief Choose the build target for hand system
 * @warning Choose "ONLY" one of them
 */

// 1. This is for testing the operation of porting a single CH101 to
// the ESP32-S3 platform. For details, please refer to the README.md of the
// folder lib/device/CH101.
#define HAND_BUILD_TARGET_PORTING_CH101

// #define HAND_BUILD_TARGET_ALL

/* USER MODIFY: Test Options End */

/*========================= ESP-IDF config ===========================*/

/* devices pin definitions */
#include "device_config.h"

#endif