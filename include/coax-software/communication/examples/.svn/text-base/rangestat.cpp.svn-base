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
			simple->cmdTimeout = 1000;
			simple->ctrlTimeout =1000;
			simple->sensorList = &sensorList;
			simple->rollCtrlMode = SB_CTRL_POS;
			simple->pitchCtrlMode = SB_CTRL_POS;
			simple->masterMode = 0;
			simple->commFreq = 60;
			// Modify here to observe a specific data set
			simple->commContent = SBS_TIMESTAMP | SBS_HRANGES;

			DEBUG(res = sbSimpleInitialise(simple));
			printf("Channel connected, continuing\n");

			return res;
		}

		int terminate() {
			res = 0;
			DEBUG(res = sbSimpleTerminate(simple));
			return res;
		}



		void acquire() {
			double rmean[4] = {0,0,0,0};
			double r2mean[4] = {0,0,0,0};
			const unsigned int n = 500;
			for (unsigned int i=0;i<n;i++) {
				if ((i % 10) == 0) {
					printf(".");fflush(stdout);
				}
				DEBUG(sbSimpleWaitState(simple,NULL,1.0));
				for (unsigned j=0;j<4;j++) {
					double r = simple->state.hranges[j];
					rmean[j] += r;
					r2mean[j] += r*r;
				}
			}
			for (unsigned j=0;j<4;j++) {
				rmean[j] /= n;
				r2mean[j] /= n;
			}
			double range=0,angle=0;
			printf("\rEnter Range and Angle: ");fflush(stdout);
			scanf(" %le %le",&range,&angle);
			printf("Ranges: %.3f ~ %.3f, %.3f ~ %.3f, %.3f ~ %.3f, %.3f ~ %.3f\n",
					rmean[0],sqrt(r2mean[0]-rmean[0]*rmean[0]),
					rmean[1],sqrt(r2mean[1]-rmean[1]*rmean[1]),
					rmean[2],sqrt(r2mean[2]-rmean[2]*rmean[2]),
					rmean[3],sqrt(r2mean[3]-rmean[3]*rmean[3]));
			FILE * fp = fopen("rcalib.txt","a");
			fprintf(fp,"%e %e %e %e %e %e %e %e %e %e\n",range,angle,
					rmean[0],sqrt(r2mean[0]-rmean[0]*rmean[0]),
					rmean[1],sqrt(r2mean[1]-rmean[1]*rmean[1]),
					rmean[2],sqrt(r2mean[2]-rmean[2]*rmean[2]),
					rmean[3],sqrt(r2mean[3]-rmean[3]*rmean[3]));
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

	SBApiSimpleContext simple;
	SBController api(&simple);

	printf("Coax basic monitoring application \n");
	CRITICAL(res = api.initialise(
				(argc<2)?("localhost"):(argv[1]), // Host name (or serial port)
				(argc<3)?("5123"):(argv[2])      // Port number
			));
	api.acquire();
	DEBUG(res = api.terminate());

	return 0;
}

		
