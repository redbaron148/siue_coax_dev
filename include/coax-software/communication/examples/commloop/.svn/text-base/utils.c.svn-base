
#include "utils.h"
#include "state.h"

#include <com/sbconst.h>
#include <com/sbchannel.h>
#include <com/sbcommloop.h>
#include <com/sbmessage.h>

#include <stdlib.h>
#include <stdio.h>


#ifdef LINUX
#include <unistd.h>
#include <time.h>
#include <assert.h>
#include <sys/time.h>
#endif

#ifdef WIN32
#include <windows.h>
#include <float.h>
#include <math.h>
#ifndef __GNUC__
#define EPOCHFILETIME (116444736000000000i64)
#else
#define EPOCHFILETIME (116444736000000000LL)
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double remainder(double x,double y)
{
	double r = fmod(x,y);
	if (r >= y/2) r -= y;
	return r;
}

double now() {
    FILETIME        ft;
    LARGE_INTEGER   li;
	__int64         t;

	GetSystemTimeAsFileTime(&ft);
	li.LowPart  = ft.dwLowDateTime;
	li.HighPart = ft.dwHighDateTime;
	t  = li.QuadPart;       /* In 100-nanosecond intervals */
	t -= EPOCHFILETIME;     /* Offset to the Epoch time */
	return t * 1e-7;
}
#define isnan _isnan
#else
double now()
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec + 1e-6*tv.tv_usec;
}

#endif

void error_display(unsigned int errorLevel, const char * message)
{
	switch (errorLevel) {
		case SB_COMMLOOP_CRITICAL:
			fprintf(stderr,"[CRITICAL] %s\n",message);
			break;
		case SB_COMMLOOP_ERROR:
			fprintf(stderr,"[ERROR] %s\n",message);
			break;
		case SB_COMMLOOP_WARNING:
			fprintf(stderr,"[WARNING] %s\n",message);
			break;
		case SB_COMMLOOP_DEBUG:
			if (gcomm.verbose) {
				fprintf(stdout,"[DEBUG] %s\n",message);
			}
			break;
		case SB_COMMLOOP_DEBUG2:
			if (gcomm.verbose>1) {
				fprintf(stdout,"[DEBUG2] %s\n",message);
			}
			break;
		case SB_COMMLOOP_OK:
		default:
			break;
	}
}

void reset(void)
{
	fprintf(stderr," **** RESET **** ");
}

// Get/Set control parameters, must return 0 on success, -1 if settings are
// ignored
int updateControlParams(int setParams, SBControlParametersMessage *params)
{
	if (setParams) {
		fprintf(stdout,"Received set param request\n");
		sbCtrlParametersPrint(stdout,params);
	} else {
		fprintf(stdout,"Received get param request\n");
		params->yawOffset = 0;
		params->baseThrust = 0;
		params->altitudeKp = 0;
		params->altitudeKi = 0;
		params->altitudeKd = 0;
		params->yawKp = 0;
		params->yawKi = 0;
		params->yawKd = 0;
		sbCtrlParametersPrint(stdout,params);
	}
	return 0;
}

// Get/Set trim mode, must return 0 on success, -1 if settings are
// ignored
int updateTrimMode(int setMode, SBTrimModeMessage *mode)
{
	if (setMode) {
		return -1; // not implemented
	} else {
		mode->trimMode = SB_TRIM_FROM_RC;
		mode->rollTrim = 0;
		mode->pitchTrim = 0;
	}
	return 0;
}


void setLightSensor(unsigned short percent) {
    printf("Light sensor set at %d percent\n",percent);
}

// Configure the bluetooth access code and name
int configureBluetooth(const char code[4], const char name[16])
{
	return -1; // not implemented
}

