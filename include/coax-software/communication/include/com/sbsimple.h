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
#ifndef SB_SIMPLE_H
#define SB_SIMPLE_H

#include "sbapi.h"

#ifdef LINUX
#include <pthread.h>
#endif

#ifdef WIN32
#include <Windows.h>
#include <Winbase.h>
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct _SBApiSimpleContext;

/** \file sbsimple.h Simplification of the helicopter API, with the minimal set
 * of function required to take-off and control the helicopter
 * */

/** 
 * Type of function called by the default state processing thread when
 * a new state has been received
 * userData is passed from SBControlContext.userData
 * */
typedef void (*SBStatePostProcessingFunc)(SBHeliState* state, void * userData);

/**
 * Type of function called once the communication has been established.
 * It is still to be defined if it can modify "context"
 * */
typedef void (*SBCommEstablishedFunc)(struct _SBApiSimpleContext *context);

/** \enum defining the type of communication desired for accessing the
 * helicopter */
typedef enum {
	/** UDP socket port, use commSpeed for port */
	SB_COMMTYPE_UDP, 
	/** unix serial port */
	SB_COMMTYPE_SERIAL, 
	/** windows serial port */
	SB_COMMTYPE_BTWIN,   
	/** unix serial port with flow control (Xbee) */
	SB_COMMTYPE_SERIAL_RTSCTS, 
} SBCommType;

/**
 * \struct _SBControlContext or \alias SBApiSimpleContext 
 * collects together all the required data to access and control the helicopter
 * */
typedef struct _SBApiSimpleContext {
	/** name of the device "COM4", "/dev/rfcomm0" */
	char device[256];              
	/** communication speed 115200 */
	unsigned int commSpeed;        
	/** communication port, when using socket, 5123 */
	unsigned int commPort;         
	/** communication type, see above */
	SBCommType commType;           
	/** \sa sbConfigureTimeout */
	unsigned short ctrlTimeout;    
	/** \sa sbConfigureTimeout */ 
	unsigned short cmdTimeout;     
	/** if true, then control will be allowed */
	unsigned char masterMode;      
	/** \sa sbConfigureOAMode */
	unsigned char oaMode;          
	/** \sa sbConfigureControl */
	unsigned char rollCtrlMode;    
	/** \sa sbConfigureControl */
	unsigned char pitchCtrlMode;   
	/** \sa sbConfigureControl */
	unsigned char yawCtrlMode;     
	/** \sa sbConfigureControl */
	unsigned char altCtrlMode;     

	/** see sbConfigureRawControl */
	unsigned char speedProfile1;   
	/** see sbConfigureRawControl */
	unsigned char speedProfile2;   

	/** desired initial navigation state. the system will do all that it can to reach it */
	unsigned char initNavState;    
	/** communication mode: on request or continuous. see sbConfigureComm */
	unsigned char commMode;        
	/** communication frequency in continuous mode. see sbConfigureComm */ 
	unsigned int  commFreq;        
	/** communication content in continuous mode. see sbConfigureComm */ 
	unsigned long commContent;     


	/** number of packet received in continuous mode. read-only */
	unsigned long packetcount;     
	/** pointer on a long where the sensor list will be written during initialization. can be null */
	unsigned long *sensorList;     
	/** internal storage for the heli state, updated in continuous mode, read-only */
	SBHeliState state;             
	/** pointer to a storage area where the heli state will be copied */
	SBHeliState *stateP;           

	/** pointer to a function that will be called once the initial 
	 * communication channel has been establised. If it is null, 
	 * then a printf-getchar interaction is used */
	SBCommEstablishedFunc commEstablishedFunc; 
	/** pointer to a function that will be called each time a state packet 
	 * is received in continuous mode. */
	SBStatePostProcessingFunc stateFunc;
	/** user-defined pointer passed to stateFunc */
	void * userData;

	/** pointer to an int that will be updated when a Ctrl-C or a 
	 * termination event is detected. read-only */
	int endFlag;
	int initialised;
	int *endP;

	/** storage for version information */
	SBVersionStatus localVersion, remoteVersion;

	/**
	 * communication details to access the helicopter. Can be used to 
	 * access advanced functions, for instance to change the control
	 * configuration after initialisation
	 * */
	struct SBControlContext control;
	/**** PRIVATE *****/
#ifdef LINUX
	pthread_t tid;
	pthread_cond_t stateCond;
	pthread_mutex_t stateMtx;
#endif
#ifdef WIN32
	HANDLE tid;
	HANDLE stateEvent;
#endif
} SBApiSimpleContext;

/***
 * Initialise the context with default values. Should be called before any 
 * use of the context variable 
 * */
int sbSimpleDefaultContext(SBApiSimpleContext *context);

/**
 * Initialise the communication, the helicopter and possibly the 
 * state reception thread, based on the information in context.
 * All configuration options are only read during this stage.
 * */
int sbSimpleInitialise(SBApiSimpleContext *context);

/**
 * Parse a target specification defined from the command line. In linux, any
 * devname starting by '/' (e.g. /dev/ttyS0, /dev/ttyUSB1) will be used as a
 * serial port. If the devname ends by ":0" or ":1", the number is used to
 * activate flow control (e.g. for the Xbee). A value of 1 makes the flow
 * control active. No value is equivalent to 0. Any other devname is used as a
 * UDP host name, and portstr is then used to extract the port, if not empty
 * (or null).
 *
 * In windows, a devname starting with "COM" will be considered as a serial
 * port. RTSCTS is not yet implemented. Anyother devname is used as a
 * UDP host name, and portstr is then used to extract the port, if not empty
 * (or null).
 *
 * */
int sbSimpleParseChannel(SBApiSimpleContext *context, 
		const char *devname, const char *portstr) ;
/**
 * Reach the desired navigation state within timeout_sec. If several
 * intermediary state are required, they will be reached one by one.
 * WARNING: this functions requires a continuous communication with at least
 * 1Hz, exporting SBS_MODES. This will be checked by waiting for a first data
 * point before requesting the transitions.
 * */
int sbSimpleReachNavState(SBApiSimpleContext *context, 
		int desired, double timeout_sec);

/**
 * Terminate the connection to the helicopter and bring it to a safe
 * configuration (landed, motor stopped)
 * */
int sbSimpleTerminate(SBApiSimpleContext *context);

/**
 * Wait for a new state to be received. Only make sense in continuous 
 * communication mode. If state is not null, the received state is copied
 * inside. If it is null, it is only copied in the internal storage, and in 
 * context->stateP pointer if not null
 * */
int sbSimpleWaitState(SBApiSimpleContext *context, SBHeliState *state, 
		double timeout_sec);

/**
 * Send a control command to the helicopter. The meaning of the command
 * depends on the control mode configuration at initialisation, if not changed
 * with sbConfigureControl
 * */
int sbSimpleControl(SBApiSimpleContext *context, 
		double roll, double pitch, double yaw,
		double altitude);

/**
 * Send a raw control command to the helicopter. Only work in RAW_COMMAND
 * control mode
 * */
int sbSimpleRawControl(SBApiSimpleContext *context, 
		double motor1, double motor2, double servo1, double servo2);

/**
 * Send a keep-alive packet to keep the communication alive, even if not
 * sending any commands
 * */
int sbSimpleKeepAlive(SBApiSimpleContext *context);

/**
 * Set the commloop verbosity, for debug. debug_channel can be:
 * DEBUG_CHANNEL_NONE, DEBUG_CHANNEL_GUMSTIX, DEBUG_CHANNEL_BLUETOOTH
 * */
int sbSimpleSetVerbose(SBApiSimpleContext *context,unsigned int verbose,
		int debug_channel);

/**
 * Configure the communication mode: frequency and content only, using
 * the simple API. Note that when using the simple API, a thread is running in
 * the background to collect data from the COAX. 
 * Continuous communication is implicit.
 * */
int sbSimpleConfigureComm(SBApiSimpleContext *context,
		unsigned int frequency, unsigned long contents);

#ifdef __cplusplus
}
#endif

#endif /* SB_SIMPLE_H */

