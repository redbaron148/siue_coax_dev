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
* Description: This software interfaces the Sharp distance sensors (analog version)
*************************************************************/

#include "sharp/sharp.h"
#include "analog/analog.h"


#define BACK	4
#define FRONT	12
#define LEFT	11
#define RIGHT	10

#define MIN_RANGE 0.0
#define MAX_RANGE 2.0


SHARP_INT_STRUCT sharp_int;
SHARP_STRUCT sharp;

static float calibrateSharp(float measurement) {
	const float pir2[4] ={
		 0,
		 0.9536743164,
		 2.532958984,
		 2.216259766
	};
	const float pir3[4] ={
		 1.53325414037879,
		-0.0113293132960221,
		 1.80266162037958,
		 2.40681781390195
	};
	const float m0 = 1200;
	const float s0 = 1000;
	float range_in_meter;
	if (measurement > 625) {
		/** this corresponds to short range measurement. In this case, the 
		 * 3rd order model is more accurate */
		float m = (measurement - m0)/s0;
		float m2 = m * m; 
		float m3 = m * m2;
		float p = pir3[0]*m3 + pir3[1]*m2 + pir3[2]*m + pir3[3];
		range_in_meter = 1./p;
	} else {
		/** at long range (>0.93m), the 2nd order model is better behaved, and
		 * gives better qualitative results. Beware that the transition is C0,
		 * not C1 */
		float m = (measurement - m0)/s0;
		float m2 = m * m; 
		float p = pir2[1]*m2 + pir2[2]*m + pir2[3];
		range_in_meter = 1./p;
	}
	if (range_in_meter < MIN_RANGE)	// saturation
		range_in_meter=MIN_RANGE;
	else if (range_in_meter > MAX_RANGE)
		range_in_meter=MAX_RANGE;	
	return range_in_meter;
}



void updateSHARP_FRONT(void)
{	
	sharp_int.front = analog_readChannel(FRONT);	// Read the Analog value of the EZ3 ultrasonic sensor	
	sharp.front = calibrateSharp(sharp_int.front);
}

void updateSHARP_LEFT(void)
{	
	sharp_int.left = analog_readChannel(LEFT);	// Read the Analog value of the EZ3 ultrasonic sensor	
	sharp.left = calibrateSharp(sharp_int.left);
}

void updateSHARP_RIGHT(void)
{	
	sharp_int.right = analog_readChannel(RIGHT);	// Read the Analog value of the EZ3 ultrasonic sensor	
	sharp.right = calibrateSharp(sharp_int.right);
}

void updateSHARP_BACK(void)
{	
	sharp_int.back = analog_readChannel(BACK);	// Read the Analog value of the EZ3 ultrasonic sensor	
	sharp.back = calibrateSharp(sharp_int.back);
}


