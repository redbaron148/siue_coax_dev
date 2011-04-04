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
#include <stdlib.h>
#include <string.h>

#include "sbsimple_util.h"

SBApiSimpleContext * sbSimpleAlloc(char * device, unsigned int port) {
	SBApiSimpleContext * result = NULL;
	result = (SBApiSimpleContext*)malloc(sizeof(SBApiSimpleContext));
	if (result) {
		sbSimpleDefaultContext(result);
		strncpy(result->device,device,255);
		result->commPort = port;
#ifdef WIN32
		int comid;
		result->commType = SB_COMMTYPE_UDP;
		if (sscanf(result->device,"COM%d",&comid)==1) {
			result->commType = SB_COMMTYPE_SERIAL;
		} else {
			result->commType = SB_COMMTYPE_UDP;
		}
#else
		if (result->device[0] == '/') {
			result->commType = SB_COMMTYPE_SERIAL;
		} else {
			result->commType = SB_COMMTYPE_UDP;
		}
#endif
		result->initNavState = SB_NAV_STOP;
		result->cmdTimeout = 30000;
		result->ctrlTimeout = 10000;
		result->altCtrlMode = SB_CTRL_REL;
		result->yawCtrlMode = SB_CTRL_VEL;
#if 0
		result->rollCtrlMode = SB_CTRL_MANUAL;// SB_CTRL_POS;
		result->pitchCtrlMode = SB_CTRL_MANUAL;// SB_CTRL_POS;
#else
		result->rollCtrlMode = SB_CTRL_POS;
		result->pitchCtrlMode = SB_CTRL_POS;
#endif
		result->oaMode = SB_OA_NONE;
	}
	return result;
}

void sbSimpleFree(SBApiSimpleContext * api) {
	if (!api->endFlag) {
		sbSimpleTerminate(api);
	}
	free(api);
}

float sbGetFloat(float * t, unsigned int i)
{
	return t[i];
}

unsigned short sbGetShort(unsigned short * t, unsigned int i)
{
	return t[i];
}

unsigned char sbGetChar(unsigned char * t, unsigned int i)
{
	return t[i];
}

