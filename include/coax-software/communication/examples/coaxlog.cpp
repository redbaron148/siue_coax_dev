/************************************************************** 
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
**************************************************************/
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

struct LogPacket
{
	unsigned long timeStamp;
	/** Current helicopter attitude */
	float roll, pitch, yaw;
	/** GYRO data */
	float gyro[3];
	/** Accelerometer data */
	float accel[3];
	float magneto[3];
	/** Range measurement in the vertical direction */
	float zrange;
	/** Output of attitude control (semantic unclear) */
	float o_attitude[3];
	/** Output of altitude control, i.e. thrust to keep the helicopter affloat  */
	float o_altitude;
	/** battery voltage */
	float battery;

	/** ranges **/
	float ranges[4];

	float rcChannel[4];

	LogPacket(const SBHeliState & state) {
		timeStamp = state.timeStamp;
		roll = state.roll;
		pitch = state.pitch;
		yaw = state.yaw;
		// zrange = state.zrange;
		o_altitude = state.o_altitude;
		battery = state.battery;
		for (unsigned int i=0;i<3;i++) {
			gyro[i] = state.gyro[i];
			accel[i] = state.accel[i];
			magneto[i] = state.magneto[i];
			o_attitude[i] = state.o_attitude[i];
		}
		// for (unsigned int i=0;i<4;i++) {
		// 	ranges[i] = state.hranges[i];
		// }
		rcChannel[0] = state.rcChannel[SB_RC_ROLL];
		rcChannel[1] = state.rcChannel[SB_RC_ROLL_TRIM];
		rcChannel[2] = state.rcChannel[SB_RC_PITCH];
		rcChannel[3] = state.rcChannel[SB_RC_PITCH_TRIM];
	}

	void serialize(FILE * fp) {
		/*            1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 15 17 18 19 20   */
		/*           ts  r  p  y g0 g1 g2 a0 a1 a2  z o1 o2 o3 oa  V r1 r2 r3 r4   */
		fprintf(fp,"%lu %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e\n",
				timeStamp,roll,pitch,yaw,gyro[0],gyro[1],gyro[2],
				accel[0],accel[1],accel[2],0.0,
				o_attitude[0],o_attitude[1],o_attitude[2],o_altitude,battery,
				magneto[0],magneto[1],magneto[2],0.0,
				rcChannel[0],rcChannel[1],rcChannel[2],rcChannel[3]);
	}

};
	

class SBController
{
	protected:
		std::vector<LogPacket> logbuffer;
		SBApiSimpleContext *simple;
		unsigned long sensorList;
		int res;
	public:
		SBController(SBApiSimpleContext *s) : simple(s), sensorList(0), res(0) {}
		~SBController() {}

		int initialise(const std::string & devname,const std::string & port="") {
			res = 0;
			logbuffer.clear();
			sbSimpleDefaultContext(simple);
			sbSimpleParseChannel(simple,devname.c_str(),port.c_str());
			simple->initNavState = SB_NAV_STOP;
			simple->cmdTimeout = 1000;
			simple->ctrlTimeout =1000;
			simple->sensorList = &sensorList;
			simple->rollCtrlMode = SB_CTRL_POS;
			simple->pitchCtrlMode = SB_CTRL_POS;
			simple->masterMode = 0;
			simple->commFreq = 100;
			simple->commContent = SBS_MODES | SBS_TIMESTAMP | SBS_RPY | SBS_BATTERY |
				SBS_GYRO | SBS_ACCEL | SBS_ALTITUDE | SBS_O_ALTITUDE | SBS_O_ATTITUDE
				;

			DEBUG(res = sbSimpleInitialise(simple));
			printf("Channel connected, continuing\n");

			return res;
		}

		int terminate() {
			res = 0;
			DEBUG(res = sbSimpleTerminate(simple));
			return res;
		}



		void log() {
			while (!*(simple->endP)) {
				DEBUG(sbSimpleWaitState(simple,NULL,1.0));
				logbuffer.push_back(LogPacket((simple->state)));
				if ((logbuffer.size()%1000) == 0) {
					printf("Buffer size %08d\n",logbuffer.size());
				}
			}
		}

		void savelog(const std::string & fname) {
			FILE * fp = fopen(fname.c_str(),"w");
			unsigned int i;
			for (i=0;i<logbuffer.size();i++) {
				logbuffer[i].serialize(fp);
			}
			fclose(fp);
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
	api.log();
	DEBUG(res = api.terminate());
	printf("Saving log\n");
	api.savelog("log.txt");
	printf("Saved log\n");

	return 0;
}

		
