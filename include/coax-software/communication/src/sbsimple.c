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
#include <stdio.h>
#include <string.h>
#include <math.h>


#ifdef LINUX
#include <pthread.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#endif

#ifdef _MSC_VER
#define inline __inline
#endif

#include "com/sbsimple.h"
#include "com/sbstate.h"
#include "com/sbversion.h"

//#define DEBUG(c) {int __res__;printf("Executing "#c"\n");__res__ = (c);printf("Result %d\n",__res__);}
#define DEBUG(c) {int __res__ = (c);if (__res__) printf("Result of "#c": %d\n",__res__);}

//#define CRITICAL(c) {int __res__;printf("Executing "#c"\n");__res__ = (c);printf("Result %d\n",__res__);if(__res__) return __res__;}
#define CRITICAL(c) {int __res__;__res__ = (c);if(__res__) {printf("Result %d\n",__res__);return __res__;}}

static int __global_end__ = 0;

#ifdef LINUX
static void sighdl(int n) {
	n = 0;
	__global_end__ ++;
	if (__global_end__>=3) {kill(getpid(),SIGTERM);}
}

SBApiSimpleContext * sbSimpleAllocContext()
{
	SBApiSimpleContext *context = malloc(sizeof(SBApiSimpleContext));
	return context;
}



void sbSimpleFreeContext(SBApiSimpleContext *context)
{
	free(context);
}


void stateCallback(SBHeliState *st, void * userData)
{
	SBApiSimpleContext *context = (SBApiSimpleContext*)userData;
	context->packetcount += 1;
	memcpy(&context->state,st,sizeof(SBHeliState));
	if (context->stateP) {
		memcpy(context->stateP,st,sizeof(SBHeliState));
	}
	if (context->stateFunc) {
		context->stateFunc(st,context->userData);
	}
	pthread_cond_broadcast(&context->stateCond);
#if 0
	printf("ping\n");
#endif
}

void * recvthread(void * _context) {
	SBApiSimpleContext *context = (SBApiSimpleContext*)_context;
	SBHeliState state;
	double lastRequestTime = -1;
	unsigned int lastFreq = -1, lastContent = 0;
	while ((__global_end__ < 2) && (context->endFlag == 0)) {
		int res;
		// We put a very small sleep here to let the control thread 
		// communicate
		if ((context->commMode!=SB_COM_CONTINUOUS) || (context->commFreq==0)) {
			usleep(1000);
			continue;
		}
		sched_yield();
		if ((sbGetCurrentTime() - lastRequestTime > 4.) 
				|| (lastContent != context->commContent)
				|| (lastFreq != context->commFreq)) {
			sbLockCommunication(&context->control);
			DEBUG(res = sbConfigureComm(&context->control, 
						context->commMode, context->commFreq, 5 * context->commFreq, context->commContent ));
			sbUnlockCommunication(&context->control);
			lastRequestTime = sbGetCurrentTime();
			lastContent = context->commContent;
			lastFreq = context->commFreq;
		}
	
		// Receive state
		sbLockCommunication(&context->control);
		res = sbWaitState(&context->control, 10); 
		if (res==0) {
			// There should be something coming, so we wait for long
			// if it is too long, it means something wrong happened with the
			// communication, and we want to trigger the watchdog
			res = sbReceiveState(&context->control,&state,1000);
		}
		sbUnlockCommunication(&context->control);
		if (res==0) {
			stateCallback(&state,context);
		} else {
			// No need to sleep, there is a sleep at the top of the loop
			// usleep(100000);
		}
		// if res is non zero, we did not receive a packet
		// this is not a problem here, it may come later
	}
	return NULL;
}

#endif


#ifdef WIN32
static BOOL WINAPI sighdl(DWORD n) {
	n = 0;
	__global_end__ ++;
	if (__global_end__>=3) {abort();}
	return TRUE;
}

void stateCallback(SBHeliState *st, void * userData)
{
	SBApiSimpleContext *context = (SBApiSimpleContext*)userData;
	context->packetcount += 1;
	memcpy(&context->state,st,sizeof(SBHeliState));
	if (context->stateP) {
		memcpy(context->stateP,st,sizeof(SBHeliState));
	}
	if (context->stateFunc) {
		context->stateFunc(st,context->userData);
	}
	PulseEvent(context->stateEvent);
}

DWORD WINAPI recvthread(LPVOID _context) {
	SBApiSimpleContext *context = (SBApiSimpleContext*)_context;
	SBHeliState state;
	double lastRequestTime = -1;
	while ((__global_end__ < 2) && (context->endFlag == 0)) {
		int res;
		// We put a very small sleep here to let the control thread 
		// communicate
		Sleep(1);
		if ((context->commFreq>0) && (sbGetCurrentTime() - lastRequestTime > 2.)) {
			sbLockCommunication(&context->control);
			DEBUG(res = sbConfigureComm(&context->control, 
						context->commMode, context->commFreq, 5 * context->commFreq, context->commContent ));
			sbUnlockCommunication(&context->control);
			lastRequestTime = sbGetCurrentTime();
		}
	
		// Receive state
		sbLockCommunication(&context->control);
		res = sbWaitState(&context->control, 10); 
		if (res==0) {
			// There should be something coming, so we wait for long
			// if it is too long, it means something wrong happened with the
			// communication, and we want to trigger the watchdog
			res = sbReceiveState(&context->control,&state,1000);
		}
		sbUnlockCommunication(&context->control);
		if (res==0) {
			stateCallback(&state,context);
		} else {
			// Sleep(100);
		}
		// if res is non zero, we did not receive a packet
		// this is not a problem here, it may come later
	}
	return 0;
}

#endif



int sbSimpleDefaultContext(SBApiSimpleContext *context)
{
#ifdef LINUX
	strcpy(context->device,"/dev/rfcomm0");
	context->commType = SB_COMMTYPE_SERIAL;
	pthread_mutex_init(&context->stateMtx,NULL);
	pthread_mutex_lock(&context->stateMtx);
	pthread_cond_init(&context->stateCond,NULL);
	context->tid = 0;
#endif
#ifdef WIN32
	char devname[] = "COM1";
	memcpy(context->device,devname,strlen(devname));
	context->commType = SB_COMMTYPE_BTWIN;
	context->tid = 0;
	context->stateEvent = CreateEvent(NULL,FALSE,FALSE,"State received");
#endif
	__global_end__ = 0;
	context->commSpeed = 115200;
	context->commPort = 5123;
	context->ctrlTimeout = 1000; // 1s
	context->cmdTimeout = 5000; // 5s
	context->masterMode = 1;
	context->oaMode = SB_OA_HORIZONTAL;
	context->rollCtrlMode = SB_CTRL_NONE;
	context->pitchCtrlMode = SB_CTRL_NONE;
	context->yawCtrlMode = SB_CTRL_VEL;
	context->altCtrlMode = SB_CTRL_POS;

	context->speedProfile1 = SB_RAWPROFILE_STEP;
	context->speedProfile2 = SB_RAWPROFILE_STEP;

	context->initNavState = SB_NAV_STOP;
	context->commMode = SB_COM_CONTINUOUS;
	context->commFreq = 10;
	context->commContent = SBS_ALL;

	context->packetcount = 0;
	context->sensorList = NULL;
	memset(&context->state,0,sizeof(SBHeliState));
	memset(&context->control,0,sizeof(struct SBControlContext));
	context->stateP = NULL;

	context->commEstablishedFunc = NULL;
	context->stateFunc = NULL;
	context->userData = NULL;
	context->endP = &(__global_end__);
	context->endFlag = 0;
	context->initialised = 0;
    // printf("Default context %p\n",context);
	
	return 0;
}


int sbSimpleInitialise(SBApiSimpleContext *context)
{
	int res;
	int commSpeed;
	const SBVersionStatus*localVersion;
#ifdef WIN32
	DWORD tid;
#endif

	sbInitialiseVersion(0); // no controller on this end of the link
	localVersion = sbGetCompiledVersion();
	context->localVersion.apiVersion = localVersion->apiVersion;
#ifdef WIN32
	strncpy_s(context->localVersion.compileTime,SB_COMPILE_TIME_LENGTH-1,
			localVersion->compileTime,SB_COMPILE_TIME_LENGTH-1);
#endif
#ifdef LINUX
	strncpy(context->localVersion.compileTime,
			localVersion->compileTime,SB_COMPILE_TIME_LENGTH-1);
#endif
	context->localVersion.compileTime[SB_COMPILE_TIME_LENGTH-1] = 0; // just in case
	printf("Local API Version\n");
	sbPrintVersion(stdout,localVersion);

	// printf("Initialising context %p\n",context);
#ifdef LINUX
	signal(SIGINT,sighdl);
#endif
#ifdef WIN32
	SetConsoleCtrlHandler(sighdl,TRUE);
#endif

	res = sbRegisterStateCallback(&context->control,stateCallback,context);

	commSpeed = context->commSpeed;
#ifdef LINUX
	switch (context->commSpeed) {
		case 9600: commSpeed = B9600; break;
		case 38400: commSpeed = B38400; break;
		case 57600: commSpeed = B57600; break;
		case 115200: commSpeed = B115200; break;
		default: break;
	}
#endif

	res = -1;
	switch (context->commType) {
		case SB_COMMTYPE_SERIAL:
			printf("Connecting to port %s speed %d\n",
					context->device,context->commSpeed);
#if defined(LINUX) && !defined(MACOSX)
			DEBUG(res = sbOpenSerialCommunication(&context->control,context->device,commSpeed,0));
#endif
			if (res) return res;
			break;
		case SB_COMMTYPE_SERIAL_RTSCTS:
			printf("Connecting to port %s speed %d with flow control\n",
					context->device,context->commSpeed);
#if defined(LINUX) && !defined(MACOSX)
			DEBUG(res = sbOpenSerialCommunication(&context->control,context->device,commSpeed,1));
#endif
			if (res) return res;
			break;
		case SB_COMMTYPE_BTWIN:
			printf("Connecting to btwin port %s speed %d\n",
					context->device,context->commSpeed);
#ifdef WIN32
			DEBUG(res = sbOpenBluetoothCommunication(&context->control,context->device,commSpeed));
#endif
			if (res) return res;
			break;
		case SB_COMMTYPE_UDP:
			printf("Connecting to host %s port %d\n",
					context->device,context->commPort);
#ifdef LINUX
			// commSpeed is the port, device is the hostname
			DEBUG(res = sbOpenSocketCommunication(&context->control,context->device,context->commPort));
#endif
#ifdef WIN32
			// commSpeed is the port, device is the hostname
			DEBUG(res = sbOpenWinsockCommunication(&context->control,context->device,context->commPort));
#endif
			if (res) return res;
			break;
		default:
			printf("Invalid communication type\n");
			return -1;
	}

	if (context->commEstablishedFunc) {
		context->commEstablishedFunc(context);
	} else {
#if 0
		// this can be useful because the initial bluetooth negociation can
		// leave the uart in a weird state, on the e-puck
		printf("Channel connected, reset platform and press enter to continue\n");
		getchar();
#endif
	}

	CRITICAL(res = sbGetVersionInformation(&(context->control),&context->remoteVersion));

	printf("Remote API Version\n");
	sbPrintVersion(stdout,&context->remoteVersion);

	if (context->remoteVersion.apiVersion != context->localVersion.apiVersion) {
		printf("Inconsistent API Version: local %04X remote %04X\n",
				context->localVersion.apiVersion,
				context->remoteVersion.apiVersion);
		return -1;
	}


	if (context->masterMode) {
		CRITICAL(res = sbConnect(&context->control));
	}
	if (context->sensorList) {
		CRITICAL(res = sbGetSensorList(&context->control, context->sensorList));
		printf("Sensor list:");
		sbContentPrint(stdout,*context->sensorList);
		printf("\n");
	}

	if (context->masterMode) {
		CRITICAL(res = sbConfigureTimeout(&context->control, context->ctrlTimeout, context->cmdTimeout ));
		CRITICAL(res = sbConfigureOAMode(&context->control, context->oaMode));
		CRITICAL(res = sbConfigureRawControl(&context->control, 
					context->speedProfile1, context->speedProfile2));
		CRITICAL(res = sbConfigureControl(&context->control, 
					context->rollCtrlMode, 
					context->pitchCtrlMode, 
					context->yawCtrlMode, 
					context->altCtrlMode));


	}

	// Defaultc communication configuration, only 5 seconds of messages are
	// requested
	CRITICAL(res = sbConfigureComm(&context->control, context->commMode, context->commFreq, 5 * context->commFreq, context->commContent ));

#ifdef WIN32
	/* TODO: Error checking */
	context->tid = CreateThread(NULL,0,recvthread,context,0,&tid);
#endif

#ifdef LINUX
	DEBUG(res = pthread_create(&context->tid,NULL,recvthread,context));
#endif

	context->initialised = 1;
	if (context->masterMode) {
		DEBUG(res = sbSimpleReachNavState(context, context->initNavState, 5.0));
	}
	switch (context->commMode) {
		case SB_COM_CONTINUOUS:
			DEBUG(res = sbSimpleWaitState(context,&context->state,2.0));
			break;
		case SB_COM_ONREQUEST:
			sbLockCommunication(&context->control);
			DEBUG(res = sbRequestState(&context->control,SBS_MODES,&context->state));
			sbUnlockCommunication(&context->control);
			break;
		default:
			res = -1;
	}
    if (!res) {
        printf("Received first state, ready to go!\n");
    }
	return res;
}

int sbSimpleParseChannel(SBApiSimpleContext *context, const char *devname, const char *portstr) 
{
#ifdef WIN32
	int comid = 0;
#endif
	context->device[0] = 0;
	context->commPort = 5123; // that is the second port when using 
	// monitoring functions for instance
	strncpy(context->device,devname,255);
#ifdef WIN32
	context->commType = SB_COMMTYPE_UDP;
	if (sscanf(context->device,"COM%d",&comid)==1) {
		context->commType = SB_COMMTYPE_SERIAL;
	} else {
		context->commType = SB_COMMTYPE_UDP;
	}
#else
	if (context->device[0] == '/') {
		int rtscts = 0;
		char dev2[256];
		if (sscanf(context->device,"%[^:]:%d",dev2,&rtscts)==2) {
			strncpy(context->device,dev2,255);
			context->commType = rtscts?SB_COMMTYPE_SERIAL_RTSCTS:SB_COMMTYPE_SERIAL;
		} else {
			context->commType = SB_COMMTYPE_SERIAL;
		}
	} else {
		context->commType = SB_COMMTYPE_UDP;
		int port = 0;
		char dev2[256];
		if (sscanf(context->device,"%[^:]:%d",dev2,&port)==2) {
			strncpy(context->device,dev2,255);
            context->commPort = port;
		}
	}
#endif
	if (portstr) {
		sscanf(portstr,"%d",&context->commPort);
	}
	context->device[255] = 0;
	return 0;
}


int sbSimpleTerminate(SBApiSimpleContext *context)
{
	int res;
	if (context->masterMode) {
		if (context->commMode == SB_COM_CONTINUOUS) {
			DEBUG(res = sbSimpleReachNavState(context, SB_NAV_STOP, 15.0));
		}
		sbLockCommunication(&context->control);
		sbConfigureComm(&context->control, SB_COM_ONREQUEST, 0,0,0);
		sbUnlockCommunication(&context->control);
	}
	__global_end__ += 2;
	context->endFlag = 1;

	printf("Stopped communication (%8X)\n",(unsigned int)context->tid);
	if (context->tid) {
#ifdef LINUX
		pthread_join(context->tid,NULL);
#endif
#ifdef WIN32
		WaitForSingleObject(context->tid,INFINITE);
		CloseHandle(context->tid);
#endif
		context->tid = 0;
		printf("Stopped receiving thread\n");
	}

	if (context->initialised) {
		DEBUG(res = sbDisconnect(&context->control));
		printf("Disconnected\n");

		sbCloseCommunication(&context->control);
		printf("Closed communication\n");
	}
	context->initialised = 0;

	return 0;
}

int sbSimpleWaitState(SBApiSimpleContext *context, SBHeliState *state, 
		double timeout_sec)
{
	int res = 0;
#ifdef LINUX
	struct timespec tsn;
#if 0
	struct timeval tv;
	gettimeofday(&tv,NULL);
	double tcur = tv.tv_sec + tv.tv_usec*1e-6 + timeout_sec;
	tsn.tv_sec = floor(tcur);
	tsn.tv_nsec = (long)((tcur - tsn.tv_sec)*1e9);
#if 1
	printf("Tinit: %d.%06ld Tout %d.%09ld\n",
			(int)tv.tv_sec,tv.tv_usec, (int)tsn.tv_sec,tsn.tv_nsec);
#endif
#else
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	double tcur = ts.tv_sec + ts.tv_nsec*1e-9 + timeout_sec;
	tsn.tv_sec = floor(tcur);
	tsn.tv_nsec = (long)((tcur - tsn.tv_sec)*1e9);
#if 0
	printf("Tinit: %d.%09ld Tout %d.%09ld\n",
			(int)ts.tv_sec,ts.tv_nsec, (int)tsn.tv_sec,tsn.tv_nsec);
#endif
#endif
	DEBUG(res = pthread_cond_timedwait(&context->stateCond,&context->stateMtx, &tsn));
	if (res != 0) {
		if (res != ETIMEDOUT) {
			perror("sbSimpleWaitState:");
		}
#if 0
	} else {
		clock_gettime(CLOCK_REALTIME, &ts);
		printf("got data: %d.%09d\n",(int)ts.tv_sec,(int)ts.tv_nsec);
#endif
	}
#endif

#ifdef WIN32
	/* TODO: Error checking */
	DEBUG(res = WaitForSingleObject(context->stateEvent,(unsigned int)(timeout_sec*1000)));
	if (res == WAIT_FAILED) {
		printf("WaitForSingleObject failed, last error %d\n",GetLastError());
	}
#endif
	if (state) {
		memcpy(state,&context->state,sizeof(SBHeliState));
	}
	return res;
}

int sbSimpleControl(SBApiSimpleContext *context,
		double roll, double pitch, double yaw,
		double altitude)
{
	int res = 0;
	sbLockCommunication(&context->control);
	DEBUG(res = sbSetControl(&context->control,roll,pitch,yaw,altitude));
	sbUnlockCommunication(&context->control);
	return 0;
}


int sbSimpleRawControl(SBApiSimpleContext *context,
		double motor1, double motor2, double servo1, double servo2)
{
	int res = 0;
	sbLockCommunication(&context->control);
	DEBUG(res = sbSetRawControl(&context->control,motor1,motor2,servo1,servo2));
	sbUnlockCommunication(&context->control);
	return 0;
}

#ifdef LINUX
static
inline double now() {
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec + 1e-6*tv.tv_usec;
}
#endif

#ifdef WIN32
#ifndef __GNUC__
#define EPOCHFILETIME (116444736000000000i64)
#else
#define EPOCHFILETIME (116444736000000000LL)
#endif

static inline double now() {
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
#endif

static inline int waitmode(int mode) {
	switch (mode){
		case SB_NAV_TAKEOFF:
			return SB_NAV_HOVER;
		case SB_NAV_LAND:
			return SB_NAV_IDLE;
		default:
			return mode;
	}
}

#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)<(b))?(a):(b))
int sbSimpleReachNavState(SBApiSimpleContext *context, 
		int desired, double timeout_sec)
{
	int res;
	SBHeliState state;
	double t0 = now();
	double t_lastprint = -2;

	static int transition[8*8] = {
	// to:  stop         idle	      takeoff         land         hover           cntrl           sink  raw     
			SB_NAV_STOP, SB_NAV_IDLE, SB_NAV_IDLE,             -1, SB_NAV_IDLE,    SB_NAV_IDLE,    -1,   SB_NAV_RAW,// from stop
			SB_NAV_STOP, SB_NAV_IDLE, SB_NAV_TAKEOFF,          -1, SB_NAV_TAKEOFF, SB_NAV_TAKEOFF, -1,   SB_NAV_STOP, // from idle
			SB_NAV_LAND, SB_NAV_LAND, SB_NAV_TAKEOFF, SB_NAV_LAND, SB_NAV_TAKEOFF, SB_NAV_TAKEOFF, -1,   SB_NAV_LAND,// from takeoff
			SB_NAV_LAND, SB_NAV_LAND, SB_NAV_LAND,    SB_NAV_LAND, SB_NAV_LAND,    SB_NAV_LAND,    -1,   SB_NAV_LAND,// from land
			SB_NAV_LAND, SB_NAV_LAND, SB_NAV_LAND,    SB_NAV_LAND, SB_NAV_HOVER,   SB_NAV_CTRLLED, -1,   SB_NAV_LAND,// from hover
			SB_NAV_LAND, SB_NAV_LAND, SB_NAV_LAND,    SB_NAV_LAND, SB_NAV_HOVER,   SB_NAV_CTRLLED, -1,   SB_NAV_LAND,// from ctrlled
			SB_NAV_LAND, SB_NAV_LAND, SB_NAV_LAND,    SB_NAV_LAND, SB_NAV_HOVER,   SB_NAV_CTRLLED, -1,   SB_NAV_LAND,// from sink
			SB_NAV_STOP, SB_NAV_STOP,          -1,    SB_NAV_LAND, SB_NAV_LAND,    SB_NAV_LAND,    -1,   SB_NAV_RAW  // from raw (no transition allowed)
	};

	switch (desired) {
		case SB_NAV_TAKEOFF:
		case SB_NAV_LAND:
			printf("sbSimpleReachNavState: Take-off and landing mode are transcient and cannot be used as target for sbSimpleReachNavState\n");
			return -1;
		case SB_NAV_SINK:
			printf("sbSimpleReachNavState: Sink mode is an error mode and cannot be used as target for sbSimpleReachNavState\n");
			return -1;
		default:
			break;
	}

	switch (context->commMode) {
		case SB_COM_CONTINUOUS:
			DEBUG(res = sbSimpleWaitState(context,&state,1.0));
			break;
		case SB_COM_ONREQUEST:
			sbLockCommunication(&context->control);
			DEBUG(res = sbRequestState(&context->control,SBS_MODES,&state));
			sbUnlockCommunication(&context->control);
			break;
		default:
			res = -1;
	}

	if (res) {
		printf("sbSimpleReachNavState: Could not receive initial state\n");
		return -1;
	}
	if (!(state.content & SBS_MODES)) {
		printf("sbSimpleReachNavState: initial state did not contains MODES\n");
		return -1;
	}
	if (desired == state.mode.navigation) {
		return 0;
	}

	while (state.mode.navigation != desired) {
		double t = now();
		int nextstate;
		switch (context->commMode) {
			case SB_COM_CONTINUOUS:
				DEBUG(res = sbSimpleWaitState(context,&state,1.0));
				break;
			case SB_COM_ONREQUEST:
				sbLockCommunication(&context->control);
				DEBUG(res = sbRequestState(&context->control,SBS_MODES,&state));
				sbUnlockCommunication(&context->control);
				break;
			default:
				res = -1;
		}
		if (res) {
			printf("sbSimpleReachNavState: failed to receive state\n");
			return -4;
		}
		nextstate = transition[state.mode.navigation*8+desired];
		// Testing the validity of the transition
		if (nextstate < 0) {
			printf("sbSimpleReachNavState: cannot reach desired state. Aborting\n");
			return -2;
		}
		// printf("sbSimpleReachNavState: next state = %d / current %d\n",nextstate, state.mode.navigation);
		if (t < t0) t += 86400;
		if (t > t0 + timeout_sec) {
			printf("sbSimpleReachNavState timed out. Aborting\n");
			return -2;
		}

		if (t - t_lastprint > 2) {
			printf("Current %s desired: %s next %s wait %s rem t %.3f\n",
					sbNavModeString(state.mode.navigation),
					sbNavModeString(desired),sbNavModeString(nextstate),
					sbNavModeString(waitmode(nextstate)),t0 + timeout_sec - t);
			t_lastprint = t;
		}

		// Requesting the transition
		sbLockCommunication(&context->control);
		res = sbSetNavMode(&context->control,nextstate);
		res = sbKeepAlive(&context->control);
		sbUnlockCommunication(&context->control);
		switch (res) {
			case 0:
				// Good, it works
				break;
			case SB_REPLY_TOO_EARLY:
				continue;
				break;
			default:
				printf("sbSetNavAndWait failed (%d) to set desired mode. Aborting\n",res);
		}

#ifdef LINUX
		usleep(100000);
#endif
#ifdef WIN32
		Sleep(100);
#endif
	}
	// At this stage, the mode is the desired mode.
	if (state.mode.navigation == SB_NAV_CTRLLED) {
		// Confirm the control mode when we reach the
		// controlled state
		sbLockCommunication(&context->control);
		res = sbConfigureControl(&context->control,
				context->rollCtrlMode, context->pitchCtrlMode, 
				context->yawCtrlMode, context->altCtrlMode);
		sbUnlockCommunication(&context->control);
	}

	return 0;
}

int sbSimpleKeepAlive(SBApiSimpleContext *context)
{
	int res;
	sbLockCommunication(&context->control);
	res = sbKeepAlive(&(context->control));
	sbUnlockCommunication(&context->control);
	return res;
}

int sbSimpleConfigureComm(SBApiSimpleContext *context,
		unsigned int frequency, unsigned long contents)
{
	context->commFreq = frequency;
	context->commContent = contents;
	return 0;
}

int sbSimpleSetVerbose(SBApiSimpleContext *context, unsigned int verbose,int debug_channel)
{
	int res;
	sbLockCommunication(&context->control);
	res = sbConfigureCommLoop(&(context->control),verbose,debug_channel);
	sbUnlockCommunication(&context->control);
	return res;
}


