#include <sys/time.h>
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
#define usleep(x) Sleep(x/1000)
#else
#include <unistd.h>
#endif

#include <com/sbapi.h>
#include <com/sbsimple.h>

#define D2R(X) ((X)*M_PI/180.0)
#define R2D(X) ((X)*180.0/M_PI)

#define DEBUG(c) res=0;c;if (res) printf("Result of "#c": %d\n",res)
#define CRITICAL(c) res=0;c;if (res) {printf("Result of "#c": %d\n",res); return res;}

int main(int argc, const char *argv[])
{
	int res=0;
	double t0;
	// Version checking
	const SBVersionStatus * compiled = sbGetCompiledVersion(); 
	unsigned int objSizeStatus = sbValidateObjectSize(compiled);
	printf("Object size status: %04x\n",objSizeStatus);
	assert(objSizeStatus == 0);


	// Configure and initialise
	SBApiSimpleContext api;
	sbSimpleDefaultContext(&api);
	// Read command line to find the type of connection
	sbSimpleParseChannel(&api,(argc>1)?argv[1]:NULL,NULL);
	DEBUG(res = sbSimpleInitialise(&api));

	// Take off to 0.3 m
	CRITICAL(res = sbSimpleReachNavState(&api,SB_NAV_RAW,30.0));
    printf("Reached RAW state\n");
	// Rotate on the spot at 18 deg/s at 0.5 m
	t0 = sbGetCurrentTime();
	while (!(*api.endP)) {
        double t = sbGetCurrentTime() - t0;
        if (t > 20.0) {
            break;
        }
		DEBUG(res = sbSimpleRawControl(&api,0.25,0.25,0.5*sin(t*M_PI/10),0.5*cos(t*M_PI/10)));
		usleep(20000);
	}
    printf("Test program completed. Switching back to STOP\n");
	// Land and shut down
	DEBUG(res = sbSimpleReachNavState(&api,SB_NAV_STOP,30.0));
	DEBUG(res = sbSimpleTerminate(&api));

	return 0;
}

		
