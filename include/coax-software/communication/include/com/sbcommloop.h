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
#ifndef SB_COMM_LOOP_H
#define SB_COMM_LOOP_H

#include "sbmessage.h"
#include "sbstate.h"
#include "sbchannel.h"


#ifdef __cplusplus
extern "C" {
#endif

#define SEND_TIMEOUT 200
#define RECEIVE_TIMEOUT 200

#define CHANNEL_0_ID 0x00
#define CHANNEL_0_FLAG 0x01
#define CHANNEL_0_NEGFLAG 0xFE

#define CHANNEL_1_ID 0x01
#define CHANNEL_1_FLAG 0x02
#define CHANNEL_1_NEGFLAG 0xFD

#ifdef PIC30
#define CHANNEL_UART_ID CHANNEL_0_ID
#define CHANNEL_BT_ID CHANNEL_1_ID
#define DEFAULT_DEBUG_CHANNEL CHANNEL_BT_ID
// #define DEFAULT_DEBUG_CHANNEL CHANNEL_UART_ID
// #define DEFAULT_DEBUG_CHANNEL -1
#else
#define DEFAULT_DEBUG_CHANNEL -1
#endif

#ifdef SIMULATION
#define DEBUG_CHANNEL -1
#define DEBUG(text) if (comm->verbose) {fprintf(stderr,"%ld-%s",comm->timecount,text);}
#define DEBUGMSG(text,msg) if (comm->verbose) {fprintf(stderr,"%ld-",comm->timecount); sbDumpMessage("",msg);}
#else
#define DEBUG(text) \
{ if (comm->debug_channel != 0xFF) { \
	char time[16]; sprintf(time,"%ld-",comm->timecount); \
	sbChannelSendString(comm->channel+comm->debug_channel,time,100); \
	sbChannelSendString(comm->channel+comm->debug_channel,text,100);}}
#define DEBUGMSG(text,msg) \
{ if (comm->debug_channel != 0xFF) { \
	char time[16]; sprintf(time,"%ld-",comm->timecount); \
	sbChannelSendString(comm->channel+comm->debug_channel,time,100); \
	sbDebugMessage(comm->channel+comm->debug_channel,text,msg);}}
#endif


#define SB_COMMLOOP_CRITICAL	0xFF
#define SB_COMMLOOP_ERROR		0x80
#define SB_COMMLOOP_WARNING		0x10
#define SB_COMMLOOP_DEBUG2		0x02
#define SB_COMMLOOP_DEBUG		0x01
#define SB_COMMLOOP_OK			0x00


typedef struct {
	// Input structures
	unsigned int verbose;
	unsigned short capacities[2];
	unsigned char debug_channel;
	SBChannel channel[2];
	void (*updateHeliState)(); // MUST not be null

	// Configuration
	int takeoffHeight_mm;
	int landingHeight_mm;
	int landingStep_mm;

	// Output structures
	unsigned long timecount;
	unsigned int timeinmode;
	unsigned int timeout_cmd;
	unsigned int timeout_ctrl;
	unsigned int cmd_watchdog;
	unsigned int ctrl_watchdog;

    // Space to store one custom message, to pass between the 2 communication
    // channels.
    SBCustomMessage customState;

	// For the function creating the message to send periodically
	SBSerialisedMessage serialisedState; 
	unsigned int stateReady[2], messageToSend[2];
	void (*setPeriodicState)(unsigned int source, unsigned int period_ms);

	// Error management
	void (*error)(unsigned int errorlevel, const char * message);

	// Reset function
	void (*reset)(void);

	// Get/Set control parameters, must return 0 on success, -1 if settings are
	// ignored
	int (*updateControlParams)(int setParams, SBControlParametersMessage *params);
	// Get/Set trim mode, must return 0 on success, -1 if settings are
	// ignored
	int (*updateTrimMode)(int setMode, SBTrimModeMessage *mode);

	// Handle blueetooth configuration message, must return 0 on success, -1
	// if settings are ignored
	int (*configureBluetooth)(const char code[4], const char name[16]);

    // Handle the coaxspeed light configuration option
    void (*setLight)(unsigned short percent);
} SBCommLoopSpec;

extern void sbCommLoopInit(SBCommLoopSpec *comm, unsigned short controllerVersion);
extern void sbCommLoop(SBCommLoopSpec *comm, SBHeliStateRaw *heliState);
extern void sbStateMachine(SBCommLoopSpec *comm, SBHeliStateRaw *heliState);
extern void sbIncrementTime(SBCommLoopSpec *comm, 
		SBHeliStateRaw *heliState, unsigned int n_ms);

#ifdef __cplusplus
}
#endif


#endif // SB_COMM_LOOP_H
