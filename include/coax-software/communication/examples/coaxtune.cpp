/************************************************************
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
************************************************************/
#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

//IO stuff
#include <sys/ioctl.h>
//#include <sys/time.h>
//#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

//
//Joystick Stuff:
#include <linux/input.h>
#include <linux/joystick.h>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef WIN32
#define sscanf sscanf_s
#define strcpy strcpy_s
#define strncpy strncpy_s
#endif

#include <string>
#include <vector>

#include <com/sbapi.h>
#include <com/sbsimple.h>
#include "PID.h"

#define D2R(X) ((X)*M_PI/180.0)
#define R2D(X) ((X)*180.0/M_PI)
#define MAX(X,Y) (((X)<(Y))?(Y):(X))
#define MIN(X,Y) (((X)<(Y))?(X):(Y))

//#define DEBUG(c) printf("Executing "#c"\n")
//#define DEBUG(c) res=0;printf("Executing "#c"\n");c;printf("Result %d\n",res)
#define DEBUG(c) res=0;c;if (res) printf("Result of "#c": %d\n",res)
#define CRITICAL(c) res=0;c;if (res) {printf("Result of "#c": %d\n",res); return res;}

static int end = 0;

void sighdl(int n) {
	end ++;
}



#define JS_DEVICE "/dev/input/js0"
#define NAME_LENGTH 128

//Value of axis is hold in an signed 16-Bit integer by the operating system
//15 bit for data and 1 bit for direction
#define joystick_max_value 32767.0


class SBController
{
	protected:
		SBApiSimpleContext *simple;
		unsigned long sensorList;
		int res;
	public:
		SBController(SBApiSimpleContext *s) : simple(s), sensorList(0), res(0) {
			sbSimpleDefaultContext(simple);
		}
		~SBController() {
		}

		int initialise(const std::string & devname) {
			res = 0;
			sbSimpleParseChannel(simple,devname.c_str(),NULL);
			simple->initNavState = SB_NAV_STOP;
			simple->cmdTimeout = 30000;
			simple->ctrlTimeout = 10000;
			simple->sensorList = &sensorList;
			simple->commFreq = 5;
			simple->altCtrlMode = SB_CTRL_MANUAL;
			simple->yawCtrlMode = SB_CTRL_MANUAL;
			simple->rollCtrlMode = SB_CTRL_MANUAL;
			simple->pitchCtrlMode = SB_CTRL_MANUAL;
			simple->oaMode = SB_OA_NONE;

			CRITICAL(res = sbSimpleInitialise(simple));
			sbLockCommunication(&simple->control);
			DEBUG(res = sbConfigureAckMode(&simple->control,0));
			sbUnlockCommunication(&simple->control);
			printf("Channel connected, continuing\n");

			return res;
		}

		int terminate() {
			res = 0;
			DEBUG(res = sbSimpleTerminate(simple));
			return res;
		}


		int load(const char * fname) {
			struct SBControlParameters params;
			res = 0;
			sbLockCommunication(&simple->control);
			DEBUG(res = sbGetControlParameters(&simple->control,&params));
			sbUnlockCommunication(&simple->control);
			if (res) return res;

			DEBUG(res = sbLoadControlParameters(fname,&params));
			if (res) return res;

			sbLockCommunication(&simple->control);
			DEBUG(res = sbSetControlParameters(&simple->control,&params));
			sbUnlockCommunication(&simple->control);
			if (res) return res;
			return 0;
		}

		int save(const char * fname) {
			struct SBControlParameters params;
			res = 0;
			sbLockCommunication(&simple->control);
			DEBUG(res = sbGetControlParameters(&simple->control,&params));
			sbUnlockCommunication(&simple->control);
			if (res) return res;

			if (strcmp(fname,"-")==0) {
				DEBUG(res = sbPrintControlParameters(stdout,&params));
			} else {
				DEBUG(res = sbSaveControlParamters(fname,&params));
			}
			if (res) return res;
			return 0;
		}

		double now() {
			struct timeval tv;
			gettimeofday(&tv,NULL);
			return tv.tv_sec + 1e-6*tv.tv_usec;
		}

};

// #define SIMULATION

int main(int argc, const char *argv[])
{
	int res;
	const SBVersionStatus * compiled = sbGetCompiledVersion(); 
	unsigned int objSizeStatus = sbValidateObjectSize(compiled);
	printf("Object size status: %04x\n",objSizeStatus);
	assert(objSizeStatus == 0);


	SBApiSimpleContext simple;
	SBController api(&simple);

	if (argc < 4) {
		printf("Usage:\n\t%s <port> <load|save> <filename>\n",argv[0]);
		return -1;
	}
	CRITICAL(res = api.initialise(argv[1]));

	if (strcasecmp(argv[2],"load")==0) {
		api.load(argv[3]);
	}
	if (strcasecmp(argv[2],"save")==0) {
		api.save(argv[3]);
	}


	DEBUG(res = api.terminate());

	return 0;
}

		
