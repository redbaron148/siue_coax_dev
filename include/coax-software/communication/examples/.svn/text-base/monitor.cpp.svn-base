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
****************************************************************/
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
            simple->commSpeed = 115200;
			simple->cmdTimeout = 1000;
			simple->ctrlTimeout =1000;
			simple->sensorList = &sensorList;
			simple->rollCtrlMode = SB_CTRL_POS;
			simple->pitchCtrlMode = SB_CTRL_POS;
			simple->masterMode = 0;
			simple->commFreq = 15;
			// Modify here to observe a specific data set
			simple->commContent = SBS_TIMESTAMP | SBS_RPY | SBS_IMU_ALL | SBS_RANGES_ALL | 
                SBS_PRESSURE | SBS_BATTERY | SBS_O_TOL | SBS_XY_REL | SBS_COAXSPEED;

			DEBUG(res = sbSimpleInitialise(simple));
			printf("Channel connected, continuing\n");

			return res;
		}

		int terminate() {
			res = 0;
			DEBUG(res = sbSimpleTerminate(simple));
			return res;
		}



		void monitor() {
			while (!*(simple->endP)) {
				DEBUG(sbSimpleWaitState(simple,NULL,1.0));
				printf("%c[2J%c[;HPacket count: %ld\n",27,27,simple->packetcount);
				sbStatePrint(stdout,&(simple->state));
			}
		}

		void testrate() {
			double t0 = sbGetCurrentTime(), dt, t1=t0, tlast=t1;
			double dtacc = 0, dt2acc = 0;
			double mean, stddev;
			unsigned int n = 0;
			while (!*(simple->endP)) {
				DEBUG(sbSimpleWaitState(simple,NULL,1.0));
				t1 = sbGetCurrentTime();
				dt = t1 - t0;
				// printf("dt: %f\n", dt);
				t0 = t1;
				dtacc += dt;
				dt2acc += dt*dt;
				n += 1;
				mean = dtacc/n;
				stddev = sqrt(dt2acc/n - mean*mean);
				if (t1 - tlast > 5.0) {
					printf("Received %d message in %f s, %f Hz, mean dt %f stddev %f voltage %f\n",
							n, t1 - tlast, n/(t1-tlast), 
							mean, stddev, simple->state.battery);
					dtacc = 0;
					dt2acc = 0;
					n = 0;
					tlast = t1;
				}
			}
		}



};



int main(int argc, const char *argv[])
{
	int res;
	const SBVersionStatus * compiled = sbGetCompiledVersion(); 
	unsigned int objSizeStatus = sbValidateObjectSize(compiled);
	printf("Object size status: %04x\n",objSizeStatus);
	assert(objSizeStatus == 0);

	int testrate = 0;

	if ((argc>1) && (strcmp(argv[1],"rate")==0)) {
		testrate = 1;
	}

	SBApiSimpleContext simple;
	SBController api(&simple);

	printf("Coax basic monitoring application \n");
	CRITICAL(res = api.initialise(
				(argc<2)?("localhost"):(argv[1]), // Host name (or serial port)
				(argc<3)?("5123"):(argv[2])      // Port number
			));
	if (res) return -1;
	if (testrate) {
		api.testrate();
	} else {
		api.monitor();
	}
	DEBUG(res = api.terminate());

	return 0;
}

		
