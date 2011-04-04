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

//#define DEBUG(c) printf("Executing "#c"\n")
//#define DEBUG(c) res=0;printf("Executing "#c"\n");c;printf("Result %d\n",res)
#define DEBUG(c) res=0;c;if (res) printf("Result of "#c": %d\n",res)
#define CRITICAL(c) res=0;c;if (res) {printf("Result of "#c": %d\n",res); return res;}

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
		SBController(SBApiSimpleContext *s) : simple(s), sensorList(0), res(0) {
			sbSimpleDefaultContext(simple);
		}
		~SBController() {
		}

		int initialise(const std::string & devname) {
			res = 0;
			sbSimpleParseChannel(simple,devname.c_str(),NULL);
			CRITICAL(res = sbSimpleInitialise(simple));
			printf("Channel connected, continuing\n");
			return res;
		}

		int terminate() {
			res = 0;
			DEBUG(res = sbSimpleTerminate(simple));
			return res;
		}


		int setMessage(const std::string & text) {
			char message[SB_STRING_MESSAGE_LENGTH];
			res = 0;
            bzero(message,SB_STRING_MESSAGE_LENGTH);
            strncpy(message,text.c_str(),std::min((size_t)(SB_STRING_MESSAGE_LENGTH-1),text.size()));
            message[SB_STRING_MESSAGE_LENGTH-1] = 0;
			sbLockCommunication(&simple->control);
			DEBUG(res = sbSetCustomMessage(&simple->control,message));
			sbUnlockCommunication(&simple->control);
			if (res) return res;
			return 0;
		}

        std::string getMessage() {
            std::string text;
			char message[SB_STRING_MESSAGE_LENGTH];
			res = 0;
			sbLockCommunication(&simple->control);
			DEBUG(res = sbGetCustomMessage(&simple->control,message));
			sbUnlockCommunication(&simple->control);
            message[SB_STRING_MESSAGE_LENGTH-1] = 0;
            if (res == 0) {
                return std::string(message);
            } else {
                return std::string();
            }
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

	if (argc < 2) {
		printf("Usage:\n\t%s [message]\n",argv[0]);
		return -1;
	}
	CRITICAL(res = api.initialise(argv[1]));

    if (argc > 2) {
        DEBUG(res = api.setMessage(argv[2]));
        if (res==0) {
            printf("Recorded message '%s'\n",argv[2]);
        }
    } else {
        std::string text = api.getMessage();
        printf("Recovered message '%s'\n",text.c_str());
    }

	DEBUG(res = api.terminate());

	return 0;
}

		
