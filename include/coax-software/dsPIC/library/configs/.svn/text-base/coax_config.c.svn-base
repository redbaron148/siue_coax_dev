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

#include "configs/coax_config.h"
#include <p33FJ256GP506.h>

#define LOOPTIMER_SCALE 156.25

void InitOscillator(void)
{

// Configure Oscillator to operate the device at 40Mhz
// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
// Fosc= 25M*40/(2*2)=80Mhz for 25M input clock

	CLKDIVbits.PLLPRE=3;				// N1=5
	CLKDIVbits.PLLPOST=0;				// N2=2
	PLLFBD=30;							// M=32
	OSCTUN=0;							// Tune FRC oscillator, if FRC is used

	RCONbits.SWDTEN=0;					// Disable Watch Dog Timer
										// clock switching to incorporate PLL
	__builtin_write_OSCCONH(0x03);		// Initiate Clock Switch to Primary
										// Oscillator with PLL (NOSC=0b011)
	__builtin_write_OSCCONL(0x01);		// Start clock switching
	while (OSCCONbits.COSC != 0b011);	// Wait for Clock switch to occur

	while(OSCCONbits.LOCK!=1) {};		// Wait for PLL to lock

}


// TIMER x functions -----------
void InitTimer1 (void)
{
	T1CON = 0x0000;             // Timer reset
 	IFS0bits.T1IF = 0;      	// Reset Timer1 interrupt flag
	IPC0bits.T1IP = 5;      	// Timer1 Interrupt priority level=4
 	TMR1=  0x0000;  	
	PR1 = 0x9c40;           	// Timer1 1ms @ 40MHz
}

void StartTimer1 (void)
{
 	IEC0bits.T1IE = 1;      	// Enable Timer1 interrupt
	T1CONbits.TON = 1;      	// Start the timer
}


void StopTimer1 (void)
{
 	IEC0bits.T1IE = 0;      	// Enable Timer1 interrupt
	T1CONbits.TON = 0;      	// Start the timer
}


// TIMER x functions -----------
void InitTimer3 (void)
{
	T3CON = 0x0000;             // Timer reset
 	IFS0bits.T3IF = 0;      	// Reset Timer1 interrupt flag
	IPC2bits.T3IP = 5;      	// Timer1 Interrupt priority level=4
 	TMR3=  0x0000;  	
//	PR3 = 0x9c40;           	// Timer1 1ms @ 40MHz
}

void StartTimer3 (void)
{
 	IEC0bits.T3IE = 1;      	// Enable Timer1 interrupt
	T3CONbits.TON = 1;      	// Start the timer
}


void StopTimer3 (void)
{
 	IEC0bits.T3IE = 0;      	// Enable Timer1 interrupt
	T3CONbits.TON = 0;      	// Start the timer
}


// TIMER 4 functions -----------

void InitLoopTimer(float ms)
{
	PR4 		=  (unsigned int) (ms*LOOPTIMER_SCALE);		// 1 thick should be 1.6us.
	T4CON 		=  0x8030;									// 1:256 Prescaler. 16Bit.
	IPC6bits.T4IP = 1;
	IFS1bits.T4IF = 0;
	IEC1bits.T4IE = 1;										// Enable Interrupt.
}

