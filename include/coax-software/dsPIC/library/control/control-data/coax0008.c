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
 
	// Control gains
	control.params.yawOffset = 0.040;	// %of top motor speed to be reduced (to reduce effort on yaw control)
	control.yaw_pid.kp = 0.03;	// 0.0433 (0.03 worked)
	control.yaw_pid.ki = 0.0006;		// 0.00586 (0.0004 worked)
	control.yaw_pid.kd = 0.0;
	// yaw.kp_heading = 0.0; 	// 0.000586 

	control.altitude_pid.kp = 0.2;
	control.altitude_pid.ki = 0.01;
	control.altitude_pid.kd = 0.1;
	control.params.Kh = 0.50; 
 
	return 0;
}
