/***********************************************************************
 * Hello Chirp! - an example application for ultrasonic sensing 
 *
 * This project is designed to be your first introduction to using
 * Chirp SonicLib to control ultrasonic sensors in an embedded C 
 * application.
 *
 * It configures connected CH101 or CH201 sensors, sets up a measurement 
 * timer, and triggers the sensors each time the timer expires.
 * On completion of each measurement, it reads out the sensor data and 
 * prints it over the console serial port.
 *
 * The settings used to configure the sensors are defined in
 * the app_config.h header file. 
 *
 ***********************************************************************/

/*
 Copyright (c) 2016-2020, Chirp Microsystems.  All rights reserved.
 Copyright (c) 2024, Dennis Liu, dennis48161025@gmail.com. All rights reserved.

 Chirp Microsystems CONFIDENTIAL

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CHIRP MICROSYSTEMS BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 You can contact the authors of this program by email at support@chirpmicro.com
 or by mail at 2560 Ninth Street, Suite 220, Berkeley, CA 94710.
*/


/* Includes */
// clang-format off
#include <stdio.h>
#include "soniclib.h"			// Chirp SonicLib sensor API definitions
#include "chirp_board_config.h"	// required header with basic device counts etc.
#include "app_config.h"
#include "app_version.h"
#include "chirp_bsp.h"			// board support package function definitions
#include "chirp_smartsonic.h"
#include "ultrasound_display_config_info.h"
// clang-format on

void app_main(void)
{

}