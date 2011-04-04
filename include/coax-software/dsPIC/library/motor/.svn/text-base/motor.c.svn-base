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
* Description:This software manages the actuators of CoaX (motors and servos) 
*************************************************************/

#include "motor/motor.h"
#include "configs/coax_config.h"
#include <p33FJ256GP506.h>

//==============================================================
//=== Configure the OUTPUT COMPARE CHANNELS 1 and 2
//==============================================================

void MOTOR_InitMotorControl(void)
{
	// Output Compare 1 -> Motor UP.
	OC1CON 		= 0x0000;
	OC1R		= 0x0000;
	OC1RS		= MOTOR_UP_INITVAL;				// 1ms.
	OC1CON		= 0x000E;						// TMR3 = Clock Source.

	// Output Compare 2 -> Motor DWN.
	OC2CON 		= 0x0000;
	OC2R		= 0x0000;
	OC2RS		= MOTOR_DWN_INITVAL;			// 1ms.
	OC2CON		= 0x000E;						// TMR3 = Clock Source.

	// Output Compare 3 -> Servo 1 (Roll)
	OC3CON 		= 0x0000;
	OC3R		= 0x0000;
	//	OC3RS		= SERVO1_INITVAL;				// Initialize to 1.5ms (servo center).
	OC3RS		= 0;							// Low state (0) signal to avoid servo noise
	OC3CON		= 0x000E;						// Timer 3.

	// Output Compare 4 -> Servo 2 (Pitch)
	OC4CON 		= 0x0000;
	OC4R		= 0x0000;
	//	OC4RS		= SERVO2_INITVAL;				// Initialize to 1.5ms (servo center).
	OC4RS		= 0;							// Low state (0) signal to avoid servo noise
	OC4CON		= 0x000E;						// Timer 3.

	//	PR3 		= 0x30D4;						// 20ms period of PPM
	PR3 		= 0x186A;						// 10ms period of PPM
	T3CON 		= 0x8020;						// 1:64 Prescaler.
}


//==============================================================
//=== Sets the motors speed
//==============================================================

void MOTOR_SetSpeed(float motorUp, float motorDown)
{
	if (motorUp > 1.0)
		motorUp  =1.0;
	if (motorUp < 0.0)
		motorUp = 0.0;
	if (motorDown > 1.0)
		motorDown =1.0;
	if (motorDown < 0.0)
		motorDown = 0.0;

	OC1RS		= (unsigned int)(PULSESCALE_A * motorUp + PULSESCALE_B);
	OC2RS		= (unsigned int)(PULSESCALE_A * motorDown + PULSESCALE_B);
}


//==============================================================
//=== Sets the servos angle
//==============================================================

void MOTOR_SetServoAngle(float servoOne, float servoTwo)
{

	if (servoOne > 1)
		servoOne =1;
	if (servoOne < -1)
		servoOne = -1;
	if (servoTwo > 1)
		servoTwo =1;
	if (servoTwo < -1)
		servoTwo = -1;

	OC3RS		= (unsigned int)(PULSESCALE_A_S * servoOne + PULSESCALE_B_S);
	OC4RS		= (unsigned int)(PULSESCALE_A_S * servoTwo + PULSESCALE_B_S);

}

//==============================================================
//=== The interruption routine
//==============================================================

void _ISR_NOAUTOPSV _T3Interrupt(void)
{
	IFS0bits.T3IF = 0;								// The Input Capture interrupt flag is cleared

}  


