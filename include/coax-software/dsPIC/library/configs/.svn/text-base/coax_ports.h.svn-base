#ifndef _COAX_PORTS
#define _COAX_PORTS
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

#include <p33FJ256GP506.h>


#define OUTPUT_PIN 0 
#define INPUT_PIN 1


//  LEDs. ---------------------------------------------------------
#define LED_ORNG 		LATBbits.LATB1
#define LED_ORNG_dir	TRISBbits.TRISB1

#define LED_RED 		LATBbits.LATB5
#define LED_RED_dir		TRISBbits.TRISB5


//  I2C. ---------------------------------------------------------
#define SIO_D		LATGbits.LATG3
#define SIO_D_dir	TRISGbits.TRISG3
#define SIO_C		LATGbits.LATG2
#define SIO_C_dir	TRISGbits.TRISG2


//  BATTERY Voltage reading (ADC). ---------------------------------
#define V_BATT_PIN		LATBbits.LATB0
#define V_BATT_PIN_dir	TRISBbits.TRISB0


//  UART. ---------------------------------------------------------
#define BT_TX_PIN		LATFbits.LATF4
#define BT_TX_PIN_dir	TRISFbits.TRISF4
#define BT_RX_PIN		LATFbits.LATF5
#define BT_RX_PIN_dir	TRISFbits.TRISF5
#define BT_RTS_PIN		LATBbits.LATB8
#define BT_RTS_PIN_dir	TRISBbits.TRISB8
#define BT_CTS_PIN		LATBbits.LATB14
#define BT_CTS_PIN_dir	TRISBbits.TRISB14



//  SPI. ---------------------------------------------------------
#define DS_SCK_2		LATGbits.LATG6
#define DS_SCK_2_dir	TRISGbits.TRISG6
#define DS_MISO_2		LATGbits.LATG7
#define DS_MISO_2_dir	TRISGbits.TRISG7
#define DS_MOSI_2		LATGbits.LATG8
#define DS_MOSI_2_dir	TRISGbits.TRISG8
#define DS_CS_2			LATGbits.LATG9		
#define DS_CS_2_dir		TRISGbits.TRISG9


//  RC. ---------------------------------------------------------

#define RC_reset		LATDbits.LATD8		
#define RC_reset_dir	TRISDbits.TRISD8	


//  BUZZER. ---------------------------------------------------------
#define BUZZER			LATBbits.LATB9
#define BUZZER_PIN_dir	TRISBbits.TRISB9


//  INT IMU. ---------------------------------------------------------
#define INT_IMU		PORTDbits.RD11
#define INT_IMU_dir	TRISDbits.TRISD11

#endif


