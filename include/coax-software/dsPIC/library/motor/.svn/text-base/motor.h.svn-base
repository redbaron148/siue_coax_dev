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
#ifndef __MOTOR_H_
#define __MOTOR_H_

/****** MOTOR CONFIG ******/
// Scaling for motor controllers.
#define PULSESCALE_A	625.0
#define PULSESCALE_B	625.0

#define MOTOR_UP_INITVAL	625
#define MOTOR_DWN_INITVAL	625

// Scaling for servos.
#define PULSESCALE_A_S	312.5	
#define PULSESCALE_B_S	937.5	

#define SERVO1_INITVAL	937		
#define SERVO2_INITVAL	937


/****** c functions ******/
void MOTOR_InitMotorControl(void);
void MOTOR_SetSpeed(float motorUp, float motorDown);
void MOTOR_SetServoAngle(float servoOne, float servoTwo);


#endif


