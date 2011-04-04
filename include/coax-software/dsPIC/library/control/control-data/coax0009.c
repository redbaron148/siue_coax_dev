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
	// Control gains with Jorge
//	yaw.offset = 0.085;	// %of top motor speed to be reduced (to reduce effort on yaw control)
//	yaw.kp_heading = 0.005;	
//	yaw.kp = 0.010;	
//	yaw.ki = 0.0045;	
//	yaw.kd = 0.0;
//
//	alt.kp = 0.15;
//	alt.ki = 0.01;
//	alt.kd = 0.1;
//	alt.kh = 0.57;

	// Control gains
	yaw.offset = 0.040;	// %of top motor speed to be reduced (to reduce effort on yaw control)
	yaw.kp = 0.0433;
	yaw.ki = 0.00586;
	yaw.kd = 0.0;
	yaw.kp_heading = 0.000586;

	alt.kp = 0.2;
	alt.ki = 0.01;
	alt.kd = 0.1;
	alt.kh = 0.52;

	return 0;
}
