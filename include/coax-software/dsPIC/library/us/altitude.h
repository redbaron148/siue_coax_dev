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
#ifndef US_ALTITUDE_H
#define US_ALTITUDE_H


//===========================================
// Exported Variables
//===========================================
//

typedef struct {
	// Raw values, in cm
	int altitudeCM;
	// Converted to meter
	float altitude, altitude_old;
	// Filtered values
	float altitude_filt;
} ALTITUDE_DATA;

extern ALTITUDE_DATA altitudeData;

//===========================================
// Exported Functions
//===========================================
//

// Initialise the altitudeData structure, and wait for the first 
// altitude measurement.
int initAltitudeSensing();

// Update the altitudeData structure 
int updateAltitudeData();


#endif // US_ALTITUDE_H
