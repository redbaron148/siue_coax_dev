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
		unsigned int naxes, nbuttons;
		int axis[32],button[32];
		int joyfd;

		pthread_t tid;
	public:
		SBController(SBApiSimpleContext *s) : simple(s), sensorList(0), res(0) {
			sbSimpleDefaultContext(simple);
			naxes = nbuttons = joyfd = -1;
			tid = -1;
		}
		~SBController() {
			if ((int)tid!=-1) {
				pthread_cancel(tid);
				pthread_join(tid,NULL);
				tid = -1;
			}
		}

		static void *joythread(void*arg) {
			SBController *ctrl = (SBController*)arg;
			while (1) {
				if (ctrl->readjoy()) break;
				usleep(10000);
			}
			return NULL;
		}

		int readjoy() {
			struct js_event js;
			if (read(joyfd, &js, sizeof(struct js_event)) != sizeof(struct js_event)) {
				return -1;
			}
			if (js.number<32) {
				switch(js.type & ~JS_EVENT_INIT){
					case JS_EVENT_BUTTON:
						button[js.number] = js.value;
						break;
					case JS_EVENT_AXIS:
						axis[js.number] = js.value;
						break;
				}
			}
			if ((*simple->endP)) {
				return -1;
			}
			return 0;
		}


		int initjoystick(const std::string & devname) {
			int version = 0;
			char name[NAME_LENGTH] ;

			if ((joyfd = open(devname.c_str(), O_RDONLY)) < 0){
				perror("Couldn't get File-Descriptor of Joystick");
				return -1;
			}

			//Now get some information about the Joystick:
			ioctl(joyfd, JSIOCGVERSION, &version);
			ioctl(joyfd, JSIOCGAXES, &naxes);
			ioctl(joyfd, JSIOCGBUTTONS, &nbuttons);
			ioctl(joyfd, JSIOCGNAME(NAME_LENGTH), name);
			naxes = naxes & 0xFF;
			nbuttons = nbuttons & 0xFF;

			printf("Joystick found:\n");
			printf("Name: %s\n",name);
			printf("Number of Buttons: %d\n",nbuttons);
			printf("Number of Axes: %d\n",naxes);
			printf("Driver-Version: %d.%d.%d\n", version>>16,
					(version>>8)&0xFF,version&0xFF);

			pthread_create(&tid,NULL,joythread,this);
			return 0;
		}


		int initialise(const std::string & devname) {
			sbSimpleParseChannel(simple,devname.c_str(),NULL);
			res = 0;

			simple->initNavState = SB_NAV_STOP;
			simple->cmdTimeout = 30000;
			simple->ctrlTimeout = 10000;
			simple->sensorList = &sensorList;
			// simple->altCtrlMode = SB_CTRL_REL;
			simple->altCtrlMode = SB_CTRL_POS;
			simple->yawCtrlMode = SB_CTRL_VEL;
			simple->commFreq = 5;
#if 0
			simple->rollCtrlMode = SB_CTRL_MANUAL;// SB_CTRL_POS;
			simple->pitchCtrlMode = SB_CTRL_MANUAL;// SB_CTRL_POS;
#else
			simple->rollCtrlMode = SB_CTRL_POS;
			simple->pitchCtrlMode = SB_CTRL_POS;
#endif
			simple->oaMode = SB_OA_NONE;

			DEBUG(res = sbSimpleInitialise(simple));
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


		double now() {
			struct timeval tv;
			gettimeofday(&tv,NULL);
			return tv.tv_sec + 1e-6*tv.tv_usec;
		}

		int joyctrl() {
			double desHeight = 0;
			bool firstctrl = true;
			while (1) {
				if ((*simple->endP)) {
					break;
				}
				// DEBUG(res = sbSimpleWaitState(simple,NULL,1.0));
				if (button[5] && (simple->state.mode.oavoid==0)) { 
					sbLockCommunication(&simple->control);
					sbConfigureOAMode(&simple->control,SB_OA_HORIZONTAL);
					sbUnlockCommunication(&simple->control);
					printf("Started obstacle avoidance\n");
				}
				if (button[6] && (simple->state.mode.oavoid!=0)) { 
					sbLockCommunication(&simple->control);
					sbConfigureOAMode(&simple->control,SB_OA_NONE);
					sbUnlockCommunication(&simple->control);
					printf("Stopped obstacle avoidance\n");
				}
				if (simple->state.errorFlags) {
					printf("An error has been detected on the PIC: %02X\n",
							simple->state.errorFlags);
					DEBUG(res = sbSimpleReachNavState(simple,SB_NAV_STOP,30.0));
					break;
				}
  
				switch (simple->state.mode.navigation) {
					case SB_NAV_STOP:
						desHeight = 0;
						firstctrl = true;
						if (button[0]) {
							DEBUG(res = sbSimpleReachNavState(simple,SB_NAV_IDLE,30.0));
						}
						break;
					case SB_NAV_IDLE:
						desHeight = 0;
						firstctrl = true;
						if (button[0]) {
							DEBUG(res = sbSimpleReachNavState(simple,SB_NAV_CTRLLED,30.0));
						}
						if (button[1]) {
							DEBUG(res = sbSimpleReachNavState(simple,SB_NAV_STOP,30.0));
						}
						break;
					case SB_NAV_TAKEOFF:
					case SB_NAV_LAND:
					case SB_NAV_HOVER:
					case SB_NAV_SINK:
						firstctrl = true;
						desHeight = simple->state.zrange;
						if (button[1]) {
							DEBUG(res = sbSimpleReachNavState(simple,SB_NAV_IDLE,30.0));
						}
						break;
					case SB_NAV_CTRLLED:
						{
							double desYaw = 0;
							double desRoll = 0;
							double desPitch = 0;
							if (firstctrl) {
								desHeight = simple->state.zrange;
								printf("Initial control: desHeight = %f\n",desHeight);
								firstctrl = false;
							}
							if (button[0]) {
								DEBUG(res = sbSimpleReachNavState(simple,SB_NAV_IDLE,30.0));
								break;
							}
							if (button[1]) { 
								desHeight -= 2e-2; 
								button[1] = 0;
								printf("Height: %f\n",desHeight);
							}
							if (button[2]) {
								desHeight += 2e-2; 
								button[2] = 0;
								printf("Height: %f\n",desHeight);
							}
							if (button[3]) { desYaw = -80*M_PI/180; }
							if (button[4]) { desYaw = +80*M_PI/180.; }
							// desYaw /= 17.; // IMU BUG
							desPitch = (0.25*axis[1]) / joystick_max_value;
							desRoll = (0.25*axis[0]) / joystick_max_value;
							DEBUG(res = sbSimpleControl(simple,desRoll,desPitch,desYaw,desHeight));
							printf("Battery: %f GyroZ: %f Joy %.3f %.3f %.3f \n",
									simple->state.battery,
									simple->state.gyro[2], 
									desHeight, simple->state.zrange, simple->state.hranges[SB_RANGE_BACK]);
							break;
						}
					default:
						break;
				}
				usleep(50000);
			}
			return 0;
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

	CRITICAL(res = api.initjoystick((argc<3)?JS_DEVICE:(argv[2])));
	CRITICAL(res = api.initialise((argc<2)?("localhost"):(argv[1])));

	DEBUG(res = api.joyctrl());

	DEBUG(res = api.terminate());

	return 0;
}

		
