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
* Description:This software is a control algorithm for the CoaX helicopter
*************************************************************/
#ifndef _CONTROL_H
#define _CONTROL_H

#include "control-tools.h"


//================================================
// DECLARATIONS
//================================================

typedef struct {
	int enable_auto; // Set to 1 when the telecommand allows autonomous ops
	int manual;      // Set by the software to enable manual control
	int velocity;    // Set by the software to enable velocity control
	float ref;
} CONTROL_DOF_SETTINGS;

typedef struct
{
	volatile float up;
	volatile float dw;
} MOTOR_SET_STRUCT;

typedef struct
{
	volatile float servo1;
	volatile float servo2;
} SERVO_SET_STRUCT;

typedef enum {
	CONTROL_STOP   = 0x01,
	CONTROL_IDLE   = 0x02,
	CONTROL_FLY    = 0x04,
	CONTROL_RAW    = 0x08,
	CONTROL_AUTO   = 0x10,
	CONTROL_MANUAL = 0x20,
	CONTROL_INTER  = 0x40
} CONTROL_MODE_ENUM;

typedef struct {
	int IMU_REBOOTED;
	int LOW_POWER_DETECTED;
} CONTROL_ERRORS;

typedef struct {
	int time_in_mode,time_in_idle;
	unsigned int CONTROL_MODE, last_CONTROL_MODE; // OR of CONTROL_MODE_ENUM
	int ABS_ALT_CTRL, last_ABS_ALT_CTRL;
	int THROTTLE; // when 1, the altitude input is the throttle
    int COUPLED;  // when 1 and the speed module is available, a simplified 
                  // model based controller is used to cancel the coupling 
                  // between the 2 servos
} CONTROL_FLAGS;

typedef struct {
	float T_ctrl_s,T_ctrl_ms;
	float Kh,Kidle;
	float yawOffset;
	float alt_min, alt_max, alt_max_sensor;
	int min_time_in_idle;
	int max_time_in_mode;
	int max_time_in_inter;
} CONTROL_PARAMS;

typedef struct {

	CONTROL_FLAGS flags;
	CONTROL_ERRORS errors;
	CONTROL_PARAMS params;

	MOTOR_SET_STRUCT motor;
	SERVO_SET_STRUCT servo;
	
	CONTROL_PID_VAR yaw_pid,altitude_pid,velocity_x_pid,velocity_y_pid;
	CONTROL_DOF_SETTINGS roll,pitch,yaw,altitude;

	// This function is called at the frequency of the control interrupt
	void (*user_function)(void *);
	void *user_context;

	unsigned short batteryvoltage;

} CONTROL_STATUS;

extern CONTROL_STATUS control;


//=============================================================
// FUNCTION PROTOTYPES: public functions
//=============================================================

// If defined, init_specific will be called at the end of the
// initialisation
void initControlVariables(void (*init_specific)(CONTROL_STATUS*));
// Request a transition of the control mode. 
// newstate in {CONTROL_STOP, CONTROL_IDLE, CONTROL_FLY}
int CONTROL_request_transition(unsigned int newstate);

//=============================================================
// FUNCTION PROTOTYPES: private functions
//=============================================================

void auto_control(void);

#endif
