/*************************************************************
*
* Low-level controller with API server for the Skybotix AG
* helicopters, and particularly COAX
*
* Developed by:
*  * Cedric Pradalier: cedric.pradalier@skybotix.ch
* Send modification or corrections as patches (diff -Naur)
* Copyright: Skybotix AG, 2009-2012
* 
* All rights reserved.
* 
* Skybotix API is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* Skybotix API is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with Skybotix API. If not, see <http://www.gnu.org/licenses/>.
* 
* 
*************************************************************/

//#include <p30f6014A.h>
#include <p33FJ256GP506.h>
#include <libpic30.h>

#include <string.h>
#include <ctype.h>
#include <stdio.h>

/* ===== CUSTOM MADE INCLUDES. ========================================== */

#include <configs/coax_config.h>
#include <configs/coax_ports.h>
#include <configs/init_port.h>
#include <utils/utils.h>
#include <led/coax_led.h>
#include <configs/coax_config.h>
#include <spi/spi.h>
#include <i2c/e_I2C_protocol.h>
#include <us/altitude.h>
#include <us/sonar.h>
#include <uart/coax_uart.h>
#include <analog/analog.h>
#include <rc/coax_rc.h>
#include <agenda/e_agenda.h>
#include <motor/motor.h>


#include "globals.h"
#include "control.h"
#include "timeouts.h"
#include "state.h"
#include "version.h"
#include "bluetooth.h"

#define MOUSE_SENSOR

#ifdef MOUSE_SENSOR
#include "mouse/mouse.h"
#endif

#ifdef COAX003X
#include "control-coax003x.h"
#endif

#ifdef COAX002X
#include "control-coax002x.h"
#endif

/* ===== CONFIGURATION BITS ============================================= */


_FBS(BWRP_WRPROTECT_OFF);							// Boot segment
_FSS(SSS_NO_SEC_CODE & SSS_NO_FLASH & SWRP_WRPROTECT_OFF);		// Secure segment

_FOSCSEL(FNOSC_PRI);								// Primary (XT, HS, EC) Oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF  & POSCMD_HS);  
// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: XT Crystanl
_FWDT(FWDTEN_OFF & WINDIS_OFF)						// Watchdog Timer
_FPOR(FPWRT_PWR128);								// POR (Power On Reset)


/* ===== DEFINITIONS. =================================================== */


int main(void) {

	// Init Oscillator. ----------------------------------------------------

	InitOscillator();			// Initialize the PLL (also disables wdt)
	__delay32(_50MILLISEC);


	// Init mcu ports ------------------------------------------------------

	init_port();    	// Initialize ports

	// Init MOTORS & SERVOS. ---------------------------------------------------------

	MOTOR_InitMotorControl();				// Must be at the beginnig to ensure proper init of MC.
    while (1) {
        MOTOR_SetSpeed(1.0, 1.0);				// Must be at the beginnig to ensure proper init of MC.
    }

	return 0;
};



