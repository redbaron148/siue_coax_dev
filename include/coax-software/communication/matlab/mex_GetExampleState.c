/*************************************************************
*
* API and Communication library for the Skybotix AG
* helicopters, and particularly COAX
*
* Developed by Cedric Pradalier: cedric.pradalier@skybotix.ch
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

#include "mex.h"
#include "SBConnection.h"
#include "utilities.h"


/* mexFunction is the gateway routine for the MEX-file. */ 
void
mexFunction( int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[] )
{
	SBHeliState S;
	checkArgs(1,0,nlhs,plhs,nrhs,prhs);


	S.errorFlags = 0x12;

	S.content = SBS_ALL;

	S.timeStamp = 1000023;
	S.controlTimeout = 1000;
	S.watchdogTimeout = 5000;

	S.mode.navigation = SB_NAV_CTRLLED;
	S.mode.communication = SB_COM_CONTINUOUS;
	S.mode.oavoid = SB_OA_VERTICAL;
	S.mode.rollAxis = SB_CTRL_NONE;
	S.mode.pitchAxis = SB_CTRL_NONE;
	S.mode.yawAxis = SB_CTRL_POS;
	S.mode.altAxis = SB_CTRL_POS;

	S.roll = 1.0;
	S.pitch = 1.1;
	S.yaw = 1.2;

	S.gyro[0] = 2.0;
	S.gyro[1] = 2.1;
	S.gyro[2] = 2.2;

	S.accel[0] = 3.0;
	S.accel[1] = 3.1;
	S.accel[2] = 3.2;

	S.magneto[0] = 4.0;
	S.magneto[1] = 4.1;
	S.magneto[2] = 4.2;

	S.hranges[0] = 5.0;
	S.hranges[1] = 5.1;
	S.hranges[2] = 5.2;

	S.imutemp = 6.0;
	S.zrange = 7.0;
	S.zfiltered = 7.5;
	S.pressure = 8.0;
	S.battery = 9.0;
	S.xrel = 10.0;
	S.yrel = 11.0;

	S.o_attitude[0] = 120;
	S.o_attitude[1] = 121;
	S.o_attitude[2] = 122;
	S.o_altitude = 130;
	S.o_tol = 140;

	S.o_xy[0] = 150;
	S.o_xy[1] = 151;

	S.o_oavoid[0] = 160;
	S.o_oavoid[1] = 161;

	plhs[0] = convertStateToMex(&S);
}




