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
* Description: This software is a set of functions to interface analog inputs
* to the dsPIC33
*************************************************************/

#include <p33FJ256GP506.h>

#include "analog.h"


void analog_initAnalog(void)
{
	AD1CON1bits.ADON = 0;		// ADC OFF.

	AD1CON1bits.ADSIDL = 0;		// Sample also in idle mode.
	AD1CON1bits.AD12B = 1;		// 12 Bit operation.
	AD1CON1bits.FORM = 0;		// Integer.
	AD1CON1bits.SSRC = 7;		// Internal counter.
	AD1CON1bits.ASAM = 0;		// Not autostart -> Set SAMP.

	AD1CON1bits.SAMP = 0;

	AD1CON1bits.DONE = 0;

	AD1CON2bits.VCFG = 0;
	AD1CON2bits.BUFM = 0;
	AD1CON2bits.ALTS = 0;

	AD1CON3bits.ADRC = 0;		// clock derived from system clock
	AD1CON3bits.SAMC = 0x1F;	// auto sample time bits 31
	AD1CON3bits.ADCS = 0x3F;	// 

	AD1PCFGH = 0xFFFF;

//	AD1CHS0bits.CH0NB = 0;
//	AD1CHS0bits.CH0SB = 5;
//	AD1CHS0bits.CH0NA = 0;
//	AD1CHS0bits.CH0SA = 5;
//
//	AD1CSSH = 0x0000;
//	AD1CSSL = 0x0010;
//
//	AD1PCFGH = 0xFFFF;
//	AD1PCFGL = 0x0010;
}

unsigned int analog_readChannel(int ichannel)		// ANichannel.
{
	unsigned int value;

	// CHANNEL 0.

	//AD1CHS0bits.CH0NB = 0;
	//AD1CHS0bits.CH0SB = ichannel;		// 0 -> AN0.
	AD1CHS0bits.CH0NA = 0;
	AD1CHS0bits.CH0SA = ichannel;

	AD1CSSH = 0x0000;					// Input scan select.
	AD1CSSL = 0x0001<<ichannel;			// 

	AD1PCFGH = 0xFFFF;					// Port configuration register. 0 -> Analog.
	AD1PCFGL = ~(0x0001<<ichannel);		//

	AD1CON1bits.ADON = 1;
	AD1CON1bits.SAMP = 1;

	// Wait for conversion completion.
	while(AD1CON1bits.DONE == 0);
	value = ADC1BUF0;

	AD1CON1bits.SAMP = 0;		// Sample enable bit, 0= ADC is holding
	AD1CON1bits.ADON = 0;		// 0= ADC module off

	return value;
}

