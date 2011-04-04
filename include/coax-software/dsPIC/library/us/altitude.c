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
* Description: This software interfaces the SRF10 sonar on the CoaX
*************************************************************/

#include <libpic30.h>

#include <p33FJ256GP506.h>
#include <led/coax_led.h>
#include "us/SRF10.h"
#include "us/altitude.h"
#include "configs/coax_config.h"
#include "configs/coax_ports.h"



ALTITUDE_DATA altitudeData;

//===========================================
// Exported Functions
//===========================================
//

int initAltitudeSensing()
{
	srf10_req_altitude();					// request the altitude for the 1st time
	__delay32(_10MILLISEC);			// remove if possible, test required !!!

	while (altitudeData.altitudeCM==0) {
		altitudeData.altitudeCM=srf10_get_altitude();	// get the altitude in [cm]
		FlashORNG ();						// Flash the LED
	}
	altitudeData.altitude=altitudeData.altitudeCM/100.0;					// conversion to [m] and to Float 
	altitudeData.altitude_old=altitudeData.altitude;
	altitudeData.altitude_filt=altitudeData.altitude;
	LED_ORNG=0;

	init_srf10_ALT();						// set max range of altitude sensor to 3m
	return 0;
}

#define absolute(x) (((x)<0)?(-(x)):(x))

int updateAltitudeData()
{
	int result = srf10_get_altitude();	// get the altitude in [cm]	  (Read I2C)


	if (result==0) {				// if the SFR10 is not responding (ranging for instance)
		srf10_req_altitude();				// request again the altitude (write I2C)
		return +1;
	}

	altitudeData.altitudeCM=result;
	altitudeData.altitude_old=altitudeData.altitude;		// uptade old altitude
	altitudeData.altitude=altitudeData.altitudeCM/100.0f;	// conversion to [m] and to Float 
//	if (absolute(altitudeData.altitude - altitudeData.altitude_old) > 0.15) {
//		altitudeData.altitude_filt = altitudeData.altitude_old;
//	} else {
		altitudeData.altitude_filt = altitudeData.altitude;
//	}

	srf10_req_altitude();				// request again the altitude (write I2C)
	return 0;
}

