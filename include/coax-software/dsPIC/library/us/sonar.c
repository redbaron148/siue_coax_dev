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
* Description: This software interfaces the MAxbotix analog sonar on the CoaX
*************************************************************/


#include "us/sonar.h"
#include "analog/analog.h"


SONAR_INT_STRUCT sonar_int;
SONAR_STRUCT sonar;



void updateSONAR_FRONT(void)
{	
	sonar_int.front = analog_readChannel(FRONT);	// Read the Analog value of the EZ3 ultrasonic sensor	
	sonar.front = CALIB_SLOPE*sonar_int.front - CALIB_SHIFT;	// change to [m]
	
	if (sonar.front < MIN_RANGE) {	// saturation
		sonar.front=MIN_RANGE;
	} else if (sonar.front > MAX_RANGE) {
		sonar.front=MAX_RANGE;	
	}
}

void updateSONAR_LEFT(void)
{	
	sonar_int.left = analog_readChannel(LEFT);	// Read the Analog value of the EZ3 ultrasonic sensor	
	sonar.left = CALIB_SLOPE*sonar_int.left - CALIB_SHIFT;	// change to [m]
	
	if (sonar.left < MIN_RANGE)	{// saturation
		sonar.left=MIN_RANGE;
	} else if (sonar.left > MAX_RANGE) {
		sonar.left=MAX_RANGE;	
	}
}

void updateSONAR_RIGHT(void)
{	
	sonar_int.right = analog_readChannel(RIGHT);	// Read the Analog value of the EZ3 ultrasonic sensor	
	sonar.right = CALIB_SLOPE*sonar_int.right - CALIB_SHIFT;	// change to [m]
	
	if (sonar.right < MIN_RANGE) {	// saturation
		sonar.right=MIN_RANGE;
	} else if (sonar.right > MAX_RANGE) { 
		sonar.right=MAX_RANGE;	
	}
}


