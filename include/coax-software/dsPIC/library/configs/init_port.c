/*************************************************************
*
* Low-level sensor and control library for the Skybotix AG
* helicopters, and particularly COAX
*
* Developed by:
*  * Samir Bouabdallah samir.bouabdallah@skybotix.ch
*  * Cedric Pradalier: cedric.pradalier@skybotix.ch
*  * Gilles Caprari: gilles.capraru@skybotix.ch
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
* Description: This software configures the dsPIC33
*************************************************************/
#include "configs/init_port.h"
#include "configs/coax_ports.h"

void init_port(void)
{

	/*LED*/
	LED_ORNG = 0;	// ORNG - RB1
	LED_ORNG_dir = OUTPUT_PIN;

	LED_RED = 0;	// RED - RB5
	LED_RED_dir = OUTPUT_PIN;
	
	/* I2C */
//	SIO_C=0;
//	SIO_D=0;
	
//	SIO_C_dir= OUTPUT_PIN;
//	SIO_D_dir= OUTPUT_PIN;

	/* SPI */
//	DS_SCK_2 = 0;
	DS_SCK_2_dir= OUTPUT_PIN;
	DS_MISO_2_dir= INPUT_PIN;
//	DS_MOSI_2 = 0;
	DS_MOSI_2_dir= OUTPUT_PIN;
//	DS_CS_2 = 1;
	DS_CS_2_dir= OUTPUT_PIN;

	/* RC */
//	RC_reset=1;
	RC_reset_dir=  OUTPUT_PIN;

	/*BATTERY*/
	V_BATT_PIN_dir = INPUT_PIN;

	/*BUZZER*/
	BUZZER=0;
	BUZZER_PIN_dir = OUTPUT_PIN;

	/* INT IMU */
	INT_IMU_dir = INPUT_PIN;
}
