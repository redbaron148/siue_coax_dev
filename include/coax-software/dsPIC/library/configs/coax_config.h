#ifndef _COAX_CONFIG
#define _COAX_CONFIG
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


/****** BASE FREQUENCY ******/
#define FCY	   40				// MIPS

/****** GENERAL CONFIG ******/
#define MICROSEC   		40uL     		//  # of cycles for 1uSec delay (minumum is 11 cycles)
#define MILLISEC   		(1000*MICROSEC)	// # of cycles for 1mSec delay (minumum is 11 cycles)

#define _1MICROSEC   	MICROSEC    		
#define _3MICROSEC   	(3*MICROSEC)    		
#define _10MICROSEC   	(10*MICROSEC)
#define _20MICROSEC   	(20*MICROSEC)
#define _30MICROSEC   	(30*MICROSEC)
#define _100MICROSEC   	(100*MICROSEC)
#define _200MICROSEC   	(200*MICROSEC)
#define _300MICROSEC   	(300*MICROSEC)
#define _500MICROSEC   	(500*MICROSEC)
#define _1MILLISEC   	MILLISEC
#define _10MILLISEC   	(10*MILLISEC)
#define _20MILLISEC   	(20*MILLISEC)
#define _25MILLISEC   	(25*MILLISEC)
#define _50MILLISEC   	(50*MILLISEC)
#define _100MILLISEC   	(100*MILLISEC)
#define _200MILLISEC   	(200*MILLISEC)
#define _500MILLISEC   	(500*MILLISEC)


#define Wait_MILLISEC   5000      		// 1mSec delay constant (only for WaitMiliSec function)
#define Wait_MICROSEC   5     			// 1uSec delay constant (only for WaitMicroSec function)

//#define	TCY_PIC		(1000000000/FCY)			// time instruction cycle in [ns]
//#define	INTERRUPT_DELAY	(10*TCY_PIC)	// delay to start an interrupt in [ns] 

#define TRUE	1
#define FALSE	0

/****** timers config ******/
//#define stop_TMR1 IEC0bits.T1IE = 0
//#define stop_TMR2 IEC0bits.T2IE = 0
//#define stop_TMR3 IEC0bits.T3IE = 0
//#define stop_TMR4 IEC1bits.T4IE = 0
//#define stop_TMR5 IEC1bits.T5IE = 0
//
//#define start_TMR1 IEC0bits.T1IE = 1
//#define start_TMR2 IEC0bits.T2IE = 1
//#define start_TMR3 IEC0bits.T3IE = 1
//#define start_TMR4 IEC1bits.T4IE = 1
//#define start_TMR5 IEC1bits.T5IE = 1


/****** c functions ******/
void InitOscillator(void);
void InitTimer1(void);
void StartTimer1 (void);
void StopTimer1 (void);


void InitTimer3(void);
void StartTimer3 (void);
void StopTimer3 (void);
void InitLoopTimer(float);


#if 1
#define _ISR_AUTOPSV __attribute__((interrupt,auto_psv))
// for interrupt without constants
#define _ISR_NOAUTOPSV __attribute__((interrupt,no_auto_psv))
#else
#define _ISR_AUTOPSV __attribute__((interrupt))
#define _ISR_NOAUTOPSV __attribute__((interrupt))
#endif


/* assembler functions */

#define Reset() {__asm__ volatile ("reset");}
#ifndef Nop
#define Nop() {__asm__ volatile ("nop");}
#endif
#define ClrWdt() {__asm__ volatile ("clrwdt");}
#define Sleep() {__asm__ volatile ("pwrsav #0");}
#define Idle() {__asm__ volatile ("pwrsav #1");}
#define InterruptOFF() {__asm__ volatile ("disi	#10000");}
#define InterruptON() {__asm__ volatile ("disi	#2");}


#endif
