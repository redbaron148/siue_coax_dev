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
* Version 0x0100: Beginning of versionning system
* Version 0x0101: Switch to mm for all heights
*************************************************************/


#include "com/sbversion.h"
#include "com/sbstate.h"
#ifndef PIC30
#include "com/sbapi.h"
#include "com/sbsimple.h"
#endif

static 
SBVersionStatus sbVersion = {
	/* apiVersion                */ 0x0200,
	/* controllerVersion         */ 0x0000,
	/* imuVersion                */ {0},
	/* compileTime               */ {0},
#ifdef PIC30
	/* sizeOf_SBApiSimpleContext */ 0, /* not def for PIC33 */
	/* sizeOf_SBControlContext   */ 0, /* not def for PIC33 */
	/* sizeOf_SBHeliState        */ 0, /* not def for PIC33 */
	/* sizeOf_SBHeliStateRaw     */ sizeof(SBHeliStateRaw),
#else
	/* sizeOf_SBApiSimpleContext */ sizeof(SBApiSimpleContext),
	/* sizeOf_SBControlContext   */ sizeof(struct SBControlContext),
	/* sizeOf_SBHeliState        */ sizeof(SBHeliState),
	/* sizeOf_SBHeliStateRaw     */ sizeof(SBHeliStateRaw),
#endif
};

void sbInitialiseVersion(unsigned short controllerVersion) 
{
	unsigned int i = 0;
	const char * compileTime = __DATE__;
	sbVersion.controllerVersion = controllerVersion;
	while ((i<SB_COMPILE_TIME_LENGTH) && compileTime[i]) {
		sbVersion.compileTime[i] = compileTime[i];
		i++;
	}
	// Pad with zero for the remaining bytes if any
	for (;i<SB_COMPILE_TIME_LENGTH;i++) {
		sbVersion.compileTime[i] = 0;
	}
	// Also pad the imu version string, just in case
	for (i=0;i<SB_IMU_VERSION_LENGTH;i++) {
		sbVersion.imuVersion[i] = 0;
	}
}

void sbSetIMUVersion(const char * imuVersion)
{
	unsigned int i = 0;
	while ((i<SB_IMU_VERSION_LENGTH) && imuVersion[i]) {
		sbVersion.imuVersion[i] = imuVersion[i];
		i++;
	}
	// Pad with zero for the remaining bytes if any
	for (;i<SB_IMU_VERSION_LENGTH;i++) {
		sbVersion.imuVersion[i] = 0;
	}
}

const SBVersionStatus * sbGetCompiledVersion() 
{
	return &sbVersion;
}


#ifdef SBC_HAS_IO
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
void sbPrintVersion(FILE * fp, const SBVersionStatus *version)
{
	char bufferIMU[SB_IMU_VERSION_LENGTH];
	char bufferCT[SB_COMPILE_TIME_LENGTH];

	fprintf(fp,"\tAPI Version: %04X\n",version->apiVersion);
	if (version->controllerVersion) {
		fprintf(fp,"\tController Version: %04X\n",version->controllerVersion);
	}
	if (version->imuVersion[0]) {
		memcpy(bufferIMU,version->imuVersion,SB_IMU_VERSION_LENGTH);
		bufferIMU[SB_IMU_VERSION_LENGTH-1] = 0;
		fprintf(fp,"\tIMU Version: %s\n",bufferIMU);
	}
	memcpy(bufferCT,version->compileTime,SB_COMPILE_TIME_LENGTH);
	bufferCT[SB_COMPILE_TIME_LENGTH-1] = 0;
	fprintf(fp,"\tCompiled on: %s\n",bufferCT);
}
#endif

