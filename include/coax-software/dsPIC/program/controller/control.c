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

#include <motor/motor.h>
#include <control/control.h>
#include <rc/coax_rc.h>
#include <led/coax_led.h>
#include <configs/coax_config.h>
#include <configs/coax_ports.h>
#include <configs/init_port.h>
#include <mouse/mouse.h>

#include <com/sbcommloop.h>
#include <com/sbconst.h>
#include <com/sbmessage.h>
#include <com/sbstate.h>
#include <com/sbchannel.h>


#include "control.h"
#include "timeouts.h"
#include "globals.h"

#define ABS(X) (((X)<0)?(-(X)):(X))

#define MAX_VOLTAGE 13000
#define MIN_VOLTAGE 11300


static int trim_mode = SB_TRIM_FROM_RC;
static float software_trim_roll = 0.;
static float software_trim_pitch = 0.;

void setSoftwareTrim(int mode, float roll, float pitch) {
	trim_mode = mode;
	if (mode == SB_TRIM_SOFTWARE) {
		software_trim_roll = roll;
		software_trim_pitch = pitch;
	}
}

int getTrimMode() {
	return trim_mode;
}

float getTrimRoll(int mode) {
	switch (mode) {
		case SB_TRIM_FROM_RC:
			return RC_ROLL_TRIM;
		case SB_TRIM_SOFTWARE:
			return software_trim_roll;
		default:
			return 0.;
	}
}

float getTrimPitch(int mode) {
	switch (mode) {
		case SB_TRIM_FROM_RC:
			return RC_PITCH_TRIM;
		case SB_TRIM_SOFTWARE:
			return software_trim_pitch;
		default:
			return 0.;
	}
}

void control_function() {

	//OMARI: update batteryvoltage for scaling of motorset point
	control.batteryvoltage = heliState.battery;

	//saturate battery readings
	if ( control.batteryvoltage > MAX_VOLTAGE ) control.batteryvoltage = MAX_VOLTAGE;
	else if (control.batteryvoltage < MIN_VOLTAGE) control.batteryvoltage = MIN_VOLTAGE;

	// call FillState
	sbIncrementTime(&gcomm,&heliState,control.params.T_ctrl_ms);
	sbStateMachine(&gcomm,&heliState);

	switch (heliState.mode.navigation) {
		case SB_NAV_RAW:
			// TODO: Implement raw control code here
			// using rawMotor1star, rawMotor2star, 
			// rawServo1star, rawServo2star and speed profiles
			// rawSpeedProfile1 and rawSpeedProfile2
			CONTROL_request_transition(CONTROL_RAW);

			MOTOR_SetSpeed(heliState.setpoint.motor1/1000., heliState.setpoint.motor2/1000.);
			MOTOR_SetServoAngle(heliState.setpoint.servo1/1000.,heliState.setpoint.servo2/1000.);
			return;
		case SB_NAV_STOP:
			CONTROL_request_transition(CONTROL_STOP);
			return;
		case SB_NAV_IDLE:
			CONTROL_request_transition(CONTROL_IDLE);
			return;
		default:
			break;
	}

	// At this point, we are sure not to be in STOP/IDLE/RAW mode
	CONTROL_request_transition(CONTROL_FLY);

	// Reset the raw input, just in case
	heliState.setpoint.motor1 = heliState.setpoint.motor2 = 0;
	heliState.setpoint.servo1 = heliState.setpoint.servo2 = 0;

	if (heliState.mode.rollAxis & SB_CTRL_MANUAL) {
		control.roll.manual = 1;
	} else {
		switch (heliState.mode.rollAxis & SB_CTRL_MANUAL_MASK) {
			case SB_CTRL_POS:
				control.roll.velocity = 0;
				control.roll.manual = 0;
				// the setpoint is in thousandth, we scale it to [-1,1]
				// Warning: plus sign to match the semantic in control/control.c
				control.roll.ref = heliState.control.roll / 1000. + 0.2*getTrimRoll(trim_mode);
				break;
			case SB_CTRL_VEL:
                if (mouse.sensor_available) {
                    control.roll.velocity = 1;
                    control.roll.manual = 0;
                    // the setpoint is in mm/s, we scale it to [-1,1]
                    // Warning: plus sign to match the semantic in control/control.c
                    control.roll.ref = heliState.control.roll / 1000.;
                } else {
                    control.roll.velocity = 0;
                    control.roll.manual = 1;
                    control.roll.ref = 0;
                }
				break;
			case SB_CTRL_NONE:
			case SB_CTRL_REL:
			case SB_CTRL_FORCE:
			default:
				control.roll.velocity = 0;
				control.roll.manual = 1;
				control.roll.ref = 0;
				break;
		}
	}

	if (heliState.mode.pitchAxis & SB_CTRL_MANUAL) {
		control.pitch.manual = 1;
	} else {
		switch (heliState.mode.pitchAxis & SB_CTRL_MANUAL_MASK) {
			case SB_CTRL_POS:
				control.pitch.manual = 0;
				control.pitch.velocity = 0;
				// the setpoint is in thousandth, we scale it to [-1,1]
				// Warning: minus sign to match the semantic in control/control.c
				control.pitch.ref = -(heliState.control.pitch / 1000. + 0.2*getTrimPitch(trim_mode));
				break;
			case SB_CTRL_VEL:
                if (mouse.sensor_available) {
                    control.pitch.manual = 0;
                    control.pitch.velocity = 1;
                    // the setpoint is in mm/s, we scale it to [-1,1]
                    // Warning: minus sign to match the semantic in control/control.c
                    control.pitch.ref = -(heliState.control.pitch / 1000.);
                } else {
                    control.pitch.velocity = 0;
                    control.pitch.manual = 1;
                    control.pitch.ref = 0;
                }
				break;
			case SB_CTRL_FORCE:
			case SB_CTRL_REL:
			case SB_CTRL_NONE:
			default:
				control.pitch.velocity = 0;
				control.pitch.manual = 1;
				control.pitch.ref = 0;
				break;
		}
	}

	float error;
	if (heliState.mode.yawAxis & SB_CTRL_MANUAL) {
		control.yaw.manual = 1;
	} else {
		switch (heliState.mode.yawAxis & SB_CTRL_MANUAL_MASK) {
			case SB_CTRL_VEL:
				control.yaw.manual = 0;
				// the setpoint is in degree/10/s, we scale to rad/s
				control.yaw.ref = heliState.control.yaw * M_PI/1800.;
				break;
			case SB_CTRL_POS:
				control.yaw.manual = 0;
				// the setpoint is in degree/10/s, we scale to rad/s
#warning	POSITION CTRL in yaw is buggy due to failing magnetometer
				error = (heliState.control.yaw - heliState.yaw)/10.;
				while (ABS(error) > 180.) {
					if (error > 180.) error-=360.;
					if (error < -180.) error+=360.;
				}

				if (error > +10.) error = +10.;
				if (error < -10.) error = -10.;
				control.yaw.ref =  0.1 *(error)/10.;
				break;
			case SB_CTRL_FORCE:
			case SB_CTRL_NONE:
			case SB_CTRL_REL:
			default:
				control.yaw.manual = 0;
				control.yaw.ref = heliState.yaw;
				break;
		}
	}

	if (heliState.mode.altAxis & SB_CTRL_MANUAL) {
		switch (heliState.mode.altAxis & SB_CTRL_MANUAL_MASK) {
			case SB_CTRL_FORCE:
				control.flags.THROTTLE = 1;
				control.altitude.manual = 1;
				break;
			case SB_CTRL_NONE:
			case SB_CTRL_REL:
			case SB_CTRL_POS:
			case SB_CTRL_VEL:
			default:
				control.flags.THROTTLE = 0;
				control.altitude.manual = 1;
				control.altitude.ref = heliState.zrange;
				break;
		}
	} else {
		switch (heliState.mode.altAxis & SB_CTRL_MANUAL_MASK) {
			case SB_CTRL_FORCE:
				control.flags.THROTTLE = 1;
				control.altitude.manual = 0;
				control.altitude.ref = heliState.control.altitude/1000.;
				break;
			case SB_CTRL_VEL:
				control.flags.THROTTLE = 0;
				control.altitude.manual = 0;
				// control.altitude.ref += (heliState.control.alt/1000.)*CTRLLOOP_PERIOD;
				control.altitude.ref = 0;
				break;
			case SB_CTRL_POS:
				control.flags.THROTTLE = 0;
				control.flags.ABS_ALT_CTRL = 1;
				control.altitude.manual = 0;
				control.altitude.ref = heliState.control.altitude/1000.;
				break;
			case SB_CTRL_REL:
				control.flags.ABS_ALT_CTRL = 0;
				control.flags.THROTTLE = 0;
				control.altitude.manual = 0;
				control.altitude.ref = heliState.control.altitude/1000.;
				break;
			case SB_CTRL_NONE:
			default:
				// not implemented
				control.flags.THROTTLE = 0;
				control.altitude.manual = 0;
				control.altitude.ref = heliState.zrange;
				break;
		}
	}
}


