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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <com/sbconst.h>
#include <com/sbmessage.h>
#include <com/sbstate.h>
#include <com/sbchannel.h>


#include "state.h"


/*************** Globals *********************/
SBHeliStateRaw heliState;
SBCommLoopSpec gcomm;

void buildCapacities(unsigned short capacity[2])
{
	capacity[0] = 0;
	capacity[1] = 0;
 	sbAddContent(capacity,SBC_MODES, SBC_TIMESTAMP, SBC_TIMEOUT,
 			SBC_ALTITUDE, SBC_HRANGES, SBC_IMU_ALL, 
 			SBC_BATTERY, SBC_CHANNELS, SBC_COAXSPEED, SBC_ALL,
            SBC_END_OF_CONTENT);
    printf("Sensor capabilities: ");
    sbContent16bPrint(stdout,capacity);
    printf("\n");
}


/******************* State managenment *******************/


void fillHeliState()
{
	heliState.errorFlags=0;
	heliState.timeStamp = gcomm.timecount; 
	heliState.controlTimeout = gcomm.timeout_ctrl; 
	heliState.watchdogTimeout = gcomm.timeout_cmd; 
	heliState.gyro[0] = 0;
	heliState.gyro[1] = 0;
	heliState.gyro[2] = 0;
	heliState.accel[0] = 0;
	heliState.accel[1] = 0;
	heliState.accel[2] = 0;
	heliState.magneto[0] = 0;
	heliState.magneto[1] = 0;
	heliState.magneto[2] = 0;
	heliState.imutemp = 0;
	heliState.zrange = 0;
	heliState.pressure = 0;
	heliState.hranges[0] = 0;
	heliState.hranges[1] = 0;
	heliState.hranges[2] = 0;
	heliState.battery = 0xFEDC;
	heliState.o_attitude[0] = 0xAABB;
	heliState.o_attitude[1] = 0xCCDD;
	heliState.o_attitude[2] = 0xEEFF;
	heliState.o_altitude = 0xDEAD;
	heliState.o_tol = 0xDEAD;
	heliState.o_xy[0] = 0xDEAD;
	heliState.o_xy[1] = 0xDEAD;
	heliState.o_oavoid[0] = 0xDEAD;
	heliState.o_oavoid[1] = 0x1234;
	memset(heliState.rcChannel,0,8*sizeof(signed short));
}


