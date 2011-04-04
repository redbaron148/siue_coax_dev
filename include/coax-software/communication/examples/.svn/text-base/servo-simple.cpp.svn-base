/****************************************************************
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
*******************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

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

//#define CRITICAL(c) res=0;printf("Executing "#c"\n");c;printf("Result %d\n",res); if(res) return res
#define CRITICAL(c) res=0;c;if (res) {printf("Result %d\n",res); return res;}

static int end = 0;

void sighdl(int n) {
	end ++;
}

class SBController
{
	protected:
		SBApiSimpleContext *simple;
		unsigned long sensorList;
		int res;
	public:
		SBController(SBApiSimpleContext *s) : simple(s), sensorList(0), res(0) {}
		~SBController() {}

		int initialise(const std::string & devname,const std::string & port="") {
			res = 0;
			sbSimpleDefaultContext(simple);
			sbSimpleParseChannel(simple,devname.c_str(),port.c_str());
			simple->initNavState = SB_NAV_STOP;
			simple->cmdTimeout = 30000;
			simple->ctrlTimeout = 10000;
			simple->sensorList = &sensorList;
			simple->altCtrlMode = SB_CTRL_REL;
			simple->yawCtrlMode = SB_CTRL_VEL;
			simple->rollCtrlMode = SB_CTRL_POS;
			simple->pitchCtrlMode = SB_CTRL_POS;
			simple->oaMode = SB_OA_NONE;

			DEBUG(res = sbSimpleInitialise(simple));
			if (res == 0) printf("Channel connected, continuing\n");

			return res;
		}

		int terminate() {
			res = 0;
			DEBUG(res = sbSimpleTerminate(simple));
			return res;
		}

		int takeoff() {
			res = 0;
			DEBUG(res = sbSimpleReachNavState(simple,SB_NAV_CTRLLED,30.0));
			printf("Take off completed\n");
			DEBUG(res = sbSimpleWaitState(simple,NULL,1.0));
			DEBUG(res = sbSimpleControl(simple,0,0,0,0.4));
			while (simple->state.zrange < 0.35) {
				DEBUG(res = sbSimpleWaitState(simple,NULL,1.0));
				DEBUG(res = sbSimpleControl(simple,0,0,0,0.4));
#ifdef WIN32
				Sleep(100);
#else
				usleep(100000);
#endif
			}
			printf("Cruising altitude reached\n");
			return res;
		}

		int yaw(double angle_deg) {
			double t1=now();
			res = 0;
			DEBUG(res = sbSimpleWaitState(simple,NULL,1.0));
			double altinit = simple->state.zrange;
			DEBUG(res = sbSimpleControl(simple,0,0,D2R(angle_deg),altinit));
			printf("Yaw: %.2f/%.2f Err %.2f\n",
					R2D(simple->state.yaw),angle_deg,
					remainder(simple->state.yaw-D2R(angle_deg),2*M_PI));
			while (fabs(remainder(simple->state.yaw-D2R(angle_deg),2*M_PI)) > D2R(2)) {
				if ((*simple->endP)) {
					break;
				}
				double error = remainder(D2R(angle_deg)-simple->state.yaw,2*M_PI);
				double Kerror = 3.0 * error;
				if (Kerror > M_PI/9) Kerror = M_PI/9;
				if (Kerror <-M_PI/9) Kerror =-M_PI/9;
				DEBUG(res = sbSimpleWaitState(simple,NULL,1.0));
				DEBUG(res = sbSimpleControl(simple,0,0,Kerror,altinit));
				double t = now();
				if (t-t1 > 1.0) {
					printf("Yaw: %.2f/%.2f Err %.2f\n",
							R2D(simple->state.yaw),angle_deg,
							remainder(simple->state.yaw-D2R(angle_deg),2*M_PI));
					t1 = t;
				}
#ifdef WIN32
				Sleep(100);
#else
				usleep(100000);
#endif
			}
			DEBUG(res = sbSimpleControl(simple,0,0,0,altinit));
			printf("Desired heading reached\n");
			return res;
		}


		int control(double distance, unsigned int sensor, double tmax) {
			double altinit = simple->state.zrange;
			double spmax = 0.5; // deg
			double t0 = now(),t1=now();
			PID pid("dctrl",0.02,spmax,0.5,0);
			pid.setImax(spmax/5);
			pid.setOmax(spmax);
			res = 0;
			while (!(*simple->endP)) {
				double pitch=0,roll=0;
				DEBUG(res = sbSimpleWaitState(simple,NULL,1.0));
				switch (sensor) {
					case SB_RANGE_FRONT:
						pitch = +pid(simple->state.hranges[sensor]-distance);
						break;
					case SB_RANGE_BACK:
						pitch = -pid(simple->state.hranges[sensor]-distance);
						break;
					case SB_RANGE_LEFT:
						roll = +pid(simple->state.hranges[sensor]-distance);
						break;
					case SB_RANGE_RIGHT:
						roll = -pid(simple->state.hranges[sensor]-distance);
						break;
					default:
						pitch = roll = 0;
						break;
				}
#ifdef WIN32
				roll = max(-spmax,min(spmax,roll));
				pitch = max(-spmax,min(spmax,pitch));
#else
				roll = std::max(-spmax,std::min(spmax,roll));
				pitch = std::max(-spmax,std::min(spmax,pitch));
#endif

				double t = now();
				if (t - t1 > 1.0) {
					printf("D %.2f Ranges: %.2f %.2f %.2f %.2f RP %.2f %.2f deg\n",distance,
							simple->state.hranges[0], simple->state.hranges[1],
							simple->state.hranges[2], simple->state.hranges[3],
							roll, pitch);
					t1 = t;
				}
				DEBUG(res = sbSimpleControl(simple,D2R(roll),D2R(pitch),0,altinit));
				if ((t - t0) > tmax) {
					break;
				}

			}
			return res;
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

	printf("Coax basic demonstration \n");
	CRITICAL(res = api.initialise(
				(argc<2)?("localhost"):(argv[1]), // Host name (or serial port)
				(argc<3)?("5123"):(argv[2])      // Port number
			));
	CRITICAL(res = api.takeoff());


	printf("Approaching\n");
	DEBUG(res = api.control(0.4 /*m*/, /*sensor */SB_RANGE_BACK, 20 /*sec*/));
	printf("Coming back\n");
	DEBUG(res = api.control(0.9 /*m*/, /*sensor */SB_RANGE_BACK, 20 /*sec*/));

#ifdef SIMULATION
	printf("Turning\n");
	DEBUG(res = api.yaw(90));
	printf("Approaching\n");
	DEBUG(res = api.control(0.4 /*m*/, /*sensor */SB_RANGE_LEFT, 20 /*sec*/));
	printf("Coming back\n");
	DEBUG(res = api.control(0.9 /*m*/, /*sensor */SB_RANGE_LEFT, 20 /*sec*/));

	printf("Turning\n");
	DEBUG(res = api.yaw(180));
	printf("Approaching\n");
	DEBUG(res = api.control(0.4 /*m*/, /*sensor */SB_RANGE_FRONT, 20 /*sec*/));
	printf("Coming back\n");
	DEBUG(res = api.control(0.9 /*m*/, /*sensor */SB_RANGE_FRONT, 20 /*sec*/));
	
	printf("Turning\n");
	DEBUG(res = api.yaw(270));
	printf("Approaching\n");
	DEBUG(res = api.control(0.4 /*m*/, /*sensor */SB_RANGE_RIGHT, 20 /*sec*/));
	printf("Coming back\n");
	DEBUG(res = api.control(0.9 /*m*/, /*sensor */SB_RANGE_RIGHT, 20 /*sec*/));

	printf("Turning\n");
	DEBUG(res = api.yaw(0));
#endif // SIMULATION

	DEBUG(res = api.terminate());

	return 0;
}

		
