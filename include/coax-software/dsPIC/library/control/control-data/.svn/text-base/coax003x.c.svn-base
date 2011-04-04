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

#include "control/control-data.h"

// WARNING: this is only used if the user of the library does not provide the
// init_specific function

int initSpecificControlParameters()
{
	// Temporary space limitation due to some noise on the sonar
	control.params.alt_max = 1.0;	// max altitude authorized [m]
	// Control gains
	control.params.yawOffset = 0.070;
	control.params.Kh = 0.50;

	control.yaw_pid.kp = 0.016; // 0.0433;
	control.yaw_pid.ki = 0.001; //0.00586;
	control.yaw_pid.kd = 0.008;

	control.altitude_pid.kp = 0.2;
	control.altitude_pid.ki = 0.002;
	control.altitude_pid.kd = 0.1;

	control.velocity_x_pid.kp = 0.2;
	control.velocity_x_pid.ki = 0.0015;
	control.velocity_x_pid.kd = 0.0;

	control.velocity_y_pid.kp = 0.2;
	control.velocity_y_pid.ki = 0.0015;
	control.velocity_y_pid.kd = 0.0;

	return 0;
}
