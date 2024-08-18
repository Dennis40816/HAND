#pragma once
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

#include "soniclib.h"            // Chirp SonicLib sensor API definitions
#include "chirp_board_config.h"  // required header with basic device counts etc.
#include "app_config.h"
#include "app_version.h"
#include "chirp_bsp.h"  // board support package function definitions
#include "chirp_smartsonic.h"
#include "ultrasound_display_config_info.h"

/* Public struct */
typedef struct hand_chirp_data_t
{
  uint32_t range;        // from ch_get_range()
  uint16_t amplitude;    // from ch_get_amplitude()
  uint16_t num_samples;  // from ch_get_num_samples()
#ifdef READ_AMPLITUDE_DATA
  uint16_t amp_data[DATA_MAX_NUM_SAMPLES];
  // from ch_get_amplitude_data()
#endif
#ifdef READ_IQ_DATA
  ch_iq_sample_t iq_data[DATA_MAX_NUM_SAMPLES];
  // from ch_get_iq_data()
#endif
} hand_chirp_data_t;