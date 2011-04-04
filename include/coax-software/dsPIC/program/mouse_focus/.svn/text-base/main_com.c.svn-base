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
#include <uart/coax_uart.h>
#include <analog/analog.h>
#include <agenda/e_agenda.h>
#include <mouse/mouse.h>


#include "mouse/mouse.h"


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





/* ===== GLOBALS. ======================================================= */

/* ===== Utilities. ========================================================== */


/* ===== MAIN. ========================================================== */


int main(void) {

	// Init Oscillator. ----------------------------------------------------

	InitOscillator();			// Initialize the PLL (also disables wdt)
	__delay32(_50MILLISEC);


	// Init mcu ports ------------------------------------------------------

	init_port();    	// Initialize ports


	// Init Analog Channels. -----------------------------------------------

	analog_initAnalog();	// Init the ADC module

	// Init SPI. ---------------------------------------------------------

	init_SPI();

	// Init I2C. ---------------------------------------------------------

	e_i2cp_init();
	e_i2cp_enable();
	__delay32(_50MILLISEC);
	e_i2c_write(0x00);	// dummy byte to get the START bit on the bus (believe it or not!)

	BuzzerBip(1,1);		// Make x bips of the Buzzer (blocking)





	// Init LED. ----------------------------------------------------------

	LED_ORNG =1;
	LED_RED = 0;


	__delay32(_200MILLISEC);  // Wait for the IMU to boot

	// Init BUZZER. ----------------------------------------------------------

	BuzzerBip(3,1);		// Make x bips of the Buzzer, blocking
    LED_ORNG = 0;
	LED_RED = 1;


	LED_RED = 0;
    //
	// Init Mouse Sensor. ----------------------------------------------------------
	mouse_init();
	mouse_send_image();//to send the image over bluetooth (stays in the function)

	return 0;
};



//====================================================
//====    START OF INTERRUPT SERVICE ROUTINES    =====
//====================================================

// TIInterrupt is declared in the coax_rc library

// T4Interrupt is declared in the control library


//=============================================================================
//Error traps
//=============================================================================

//====================================
//Oscillator Fail Error trap routine
//====================================

void _ISR_AUTOPSV _OscillatorFail(void)
{
	INTCON1bits.OSCFAIL=0;		// fail flag clearing
	__asm__ volatile ("reset");
}

//====================================
//Address Error trap routine
//====================================

void _ISR_AUTOPSV _AddressError(void) 	
{	
	INTCON1bits.ADDRERR=0;		// fail flag clearing
	__asm__ volatile ("reset");
}

//====================================
//Stack Error trap routine
//====================================

void _ISR_AUTOPSV _StackError(void)		
{		
	INTCON1bits.STKERR=0;		// fail flag clearing
	__asm__ volatile ("reset");
}

//====================================
//Math (Arithmetic) Error trap routine
//====================================

void _ISR_AUTOPSV _MathError(void)		
{	
	INTCON1bits.MATHERR=0;		// fail flag clearing
	__asm__ volatile ("reset");
}

