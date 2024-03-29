/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
#include "ultrasound_display_config_info.h"

uint8_t ultrasound_display_config_info(ch_dev_t *dev_ptr)
{
	ch_config_t 	read_config;
	uint8_t 		chirp_error;
	uint8_t 		dev_num = ch_get_dev_num(dev_ptr);

	/* Read configuration values for the device into ch_config_t structure */
	chirp_error = ch_get_config(dev_ptr, &read_config);

	if (!chirp_error) {
		const char *mode_string;

		switch (read_config.mode) {
			case CH_MODE_IDLE:
				mode_string = "IDLE";
				break;
			case CH_MODE_FREERUN:
				mode_string = "FREERUN";
				break;
			case CH_MODE_TRIGGERED_TX_RX:
				mode_string = "TRIGGERED_TX_RX";
				break;
			case CH_MODE_TRIGGERED_RX_ONLY:
				mode_string = "TRIGGERED_RX_ONLY";
				break;
			default:
				mode_string = "UNKNOWN";
		}

		/* Display sensor number, mode and max range */
		printf("Sensor %d:\tmax_range=%dmm \tmode=%s  ", dev_num,
				read_config.max_range, mode_string);

		/* Display static target rejection range, if used */
		if (read_config.static_range != 0) {
			printf("static_range=%d samples", read_config.static_range);
		}

		/* Display detection thresholds (only supported on CH201) */
		if (ch_get_part_number(dev_ptr) == CH201_PART_NUMBER) {
			ch_thresholds_t read_thresholds;

			/* Get threshold values in structure */
			chirp_error = ch_get_thresholds(dev_ptr, &read_thresholds);

			if (!chirp_error) {
				printf("\n  Detection thresholds:\n");
				for (int i = 0; i < CH_NUM_THRESHOLDS; i++) {
					printf("     %d\tstart: %2d\tlevel: %d\n", i,
							read_thresholds.threshold[i].start_sample,
							read_thresholds.threshold[i].level);
				}
			}
		}
		printf("\n");

	} else {
		printf(" Device %d: Error during ch_get_config()\n", dev_num);
	}

	return chirp_error;
}