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
#ifndef SKYBOTICS_COM_H
#define SKYBOTICS_COM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "com/sbconst.h"

/*
 * Skybotics message definition interface
 * Author: C. Pradalier
 *
 * */


// 102 is the maximum message size identified
#define SB_MAX_MESSAGE_SIZE 110
#define SB_MESSAGE_HEADER_LEN 4
#define SB_MESSAGE_START_FRAME 0xCD

typedef struct {
	unsigned char msgid;
	unsigned char handle;
	unsigned char len;
	// bytes 3-105 will be usable to store message body data
	unsigned char data[SB_MAX_MESSAGE_SIZE]; 
	unsigned char crc;
} SBSerialisedMessage;

#include "com/sbversion.h"

int sbInitializeMessage(SBSerialisedMessage *sm, 
		unsigned char msgid, unsigned char handle);

int sbFinalizeMessage(SBSerialisedMessage *sm);

int sbValidateMessage(SBSerialisedMessage *sm);

int sbDecodeMessage(SBSerialisedMessage *sm, void *dest);


#ifdef SBC_HAS_IO
void sbDumpMessage(const char * prefix, const SBSerialisedMessage *sm);
#endif

#ifdef SBC_HAS_COMM

#include "com/sbchannel.h"

// Wait a message of type msgid. Any other message is discarded
// If msgid is < 0, the first valid message is returned
int sbWaitRawMessage(SBChannel *channel, int msgid, SBSerialisedMessage *sm, unsigned int timeout_ms);

// Wait a specific message of type msgid, and decode it in dest. 
// Obviously, dest must have the right type. No checking is performed.
int sbWaitMessage(SBChannel *channel, unsigned char msgid, void *dest, unsigned int timeout_ms);

int sbSendMessage(SBChannel *channel, SBSerialisedMessage *sm, unsigned int timeout_ms);

void sbDebugMessage(SBChannel *channel,
		const char * prefix, SBSerialisedMessage *sm);

#endif


// COMMAND MESSAGES
//

// No content so far in the following 4 messages
// typedef struct {
// } SBConnectMessage;
// 
// typedef struct {
// } SBDisconnectMessage;
// 
// typedef struct {
// } SBRequestSensorList;
// 
// typedef struct {
// } SBKeepAlive;
// 
//
// typedef struct {
// } SBReset;
// 
//

#if SB_STRING_MESSAGE_LENGTH > (SB_MAX_MESSAGE_SIZE-3)
#error SB_STRING_MESSAGE_LENGTH is longer than possible
#endif

typedef struct {
	unsigned char text[SB_STRING_MESSAGE_LENGTH];
} SBStringMessage;

int sbStringMsgEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBStringMessage* text);
int sbStringMsgDecode(const SBSerialisedMessage *msg, SBStringMessage* text);

typedef struct {
	unsigned char text[SB_STRING_MESSAGE_LENGTH];
} SBCustomMessage;

int sbCustomMsgEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBCustomMessage* data, int reply);
int sbCustomMsgDecode(const SBSerialisedMessage *msg, SBCustomMessage* data);

#if SB_COMPILE_TIME_LENGTH > (SB_MAX_MESSAGE_SIZE-7)
#error SB_COMPILE_TIME_LENGTH is longer than possible
#endif

int sbVersionMsgEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBVersionStatus* version);
int sbVersionMsgDecode(const SBSerialisedMessage *msg, SBVersionStatus* version);

typedef struct {
	unsigned char verbosity;
	unsigned char debug_channel;
} SBConfigureCommLoop;
#ifndef PIC30
int sbCfgCommLoopEncode(SBSerialisedMessage *msg, unsigned char handle, const SBConfigureCommLoop* state, int requestAck);
#endif
int sbCfgCommLoopDecode(const SBSerialisedMessage *msg, SBConfigureCommLoop* state);

typedef struct {
	unsigned char oavoidMode;
} SBConfigureObstAvoid;
#ifndef PIC30
int sbCfgOAEncode(SBSerialisedMessage *msg, unsigned char handle, const SBConfigureObstAvoid* state, int requestAck);
#endif
int sbCfgOADecode(const SBSerialisedMessage *msg, SBConfigureObstAvoid* state);

typedef struct {
	unsigned char percent;
} SBCoaxSpeedSetLight;
#ifndef PIC30
int sbCoaxSpeedSetLightEncode(SBSerialisedMessage *msg, unsigned char handle, const SBCoaxSpeedSetLight* setlight, int requestAck);
#endif
int sbCoaxSpeedSetLightDecode(const SBSerialisedMessage *msg, SBCoaxSpeedSetLight* setlight);

typedef struct {
	unsigned char commMode;
	unsigned char frequency;
	unsigned short numMessages;
	unsigned short content[2];
} SBConfigureCommunication;
#ifndef PIC30
int sbCfgCommEncode(SBSerialisedMessage *msg, unsigned char handle, const SBConfigureCommunication* state, int requestAck);
#endif
int sbCfgCommDecode(const SBSerialisedMessage *msg, SBConfigureCommunication* state);


typedef struct {
	unsigned char roll;
	unsigned char pitch;
	unsigned char yaw;
	unsigned char altitude;
} SBConfigureControl;
#ifndef PIC30
int sbCfgControlEncode(SBSerialisedMessage *msg, unsigned char handle, const SBConfigureControl* state, int requestAck);
#endif
int sbCfgControlDecode(const SBSerialisedMessage *msg, SBConfigureControl* state);

typedef struct {
	unsigned short controlTimeout;
	unsigned short watchdogTimeout;
} SBConfigureTimeout;
#ifndef PIC30
int sbCfgTimeoutEncode(SBSerialisedMessage *msg, unsigned char handle, const SBConfigureTimeout* state, int requestAck);
#endif
int sbCfgTimeoutDecode(const SBSerialisedMessage *msg, SBConfigureTimeout* state);

typedef struct {
	unsigned short content[2];
} SBRequestState;
#ifndef PIC30
int sbReqStateEncode(SBSerialisedMessage *msg, unsigned char handle, const SBRequestState* state);
#endif
int sbReqStateDecode(const SBSerialisedMessage *msg, SBRequestState* state);

typedef struct {
	unsigned char mode;
} SBSetNavigationMode;
#ifndef PIC30
int sbSetNavEncode(SBSerialisedMessage *msg, unsigned char handle, const SBSetNavigationMode* state, int requestAck);
#endif
int sbSetNavDecode(const SBSerialisedMessage *msg, SBSetNavigationMode* state);

typedef struct {
	unsigned short roll;
	unsigned short pitch;
	unsigned short yaw;
	unsigned short altitude;
} SBSetControl;
#ifndef PIC30
int sbSetControlEncode(SBSerialisedMessage *msg, unsigned char handle, const SBSetControl* state, int requestAck);
#endif
int sbSetControlDecode(const SBSerialisedMessage *msg, SBSetControl* state);

typedef struct {
	unsigned short roll;
	unsigned short pitch;
	unsigned short yaw;
	unsigned short altitude;
	unsigned long timestamp;
} SBSetControlWithTimestamp;
#ifndef PIC30
int sbSetControlWithTimestampEncode(SBSerialisedMessage *msg, unsigned char handle, const SBSetControlWithTimestamp* state, int requestAck);
#endif
int sbSetControlWithTimestampDecode(const SBSerialisedMessage *msg, SBSetControlWithTimestamp* state);

typedef struct {
	unsigned short motor1;
	unsigned short motor2;
	unsigned short servo1;
	unsigned short servo2;
} SBRawControl;
#ifndef PIC30
int sbRawControlEncode(SBSerialisedMessage *msg, unsigned char handle, const SBRawControl* state, int requestAck);
#endif
int sbRawControlDecode(const SBSerialisedMessage *msg, SBRawControl* state);

typedef struct {
	unsigned char speedprofile1;
	unsigned char speedprofile2;
} SBConfigureRawControl;
#ifndef PIC30
int sbCfgRawControlEncode(SBSerialisedMessage *msg, unsigned char handle, const SBConfigureRawControl* cfg, int requestAck);
#endif
int sbCfgRawControlDecode(const SBSerialisedMessage *msg, SBConfigureRawControl* cfg);

typedef struct {
	unsigned char trimMode; // defined in sbconst.h
	// All position multiplied by 10000
	signed short rollTrim;
	signed short pitchTrim;
} SBTrimModeMessage;
int sbTrimModeEncode(SBSerialisedMessage *msg, unsigned char handle, const SBTrimModeMessage* cfg, int setRequest);
int sbTrimModeDecode(const SBSerialisedMessage *msg, SBTrimModeMessage* cfg);

/** Provide a string representation of constant SB_TRIM_xxxx */
const char* sbTrimModeString(unsigned char mode);


typedef struct {
	// All gains multiplied by 1000
	unsigned short baseThrust; // kh
	unsigned short yawOffset; 
	signed short altitudeKp;
	signed short altitudeKi;
	signed short altitudeKd;
	signed short yawKp;
	signed short yawKi;
	signed short yawKd;
} SBControlParametersMessage;
int sbCtrlParametersEncode(SBSerialisedMessage *msg, unsigned char handle, const SBControlParametersMessage* cfg, int setRequest);
int sbCtrlParametersDecode(const SBSerialisedMessage *msg, SBControlParametersMessage* cfg);

typedef struct {
	// All position in mm
	signed long x,y,z;
	// All angles scaled so that Pi = 0x8000
	unsigned short roll;
	unsigned short pitch;
	unsigned short yaw;
} SB6DofPoseMessage;
int sb6DofPoseEncode(SBSerialisedMessage *msg, unsigned char handle, const SB6DofPoseMessage* pose, int reply);
int sb6DofPoseDecode(const SBSerialisedMessage *msg, SB6DofPoseMessage* pose);

typedef struct {
	char code[4];
	char name[16];
} SBConfigureBluetoothMessage;
int sbConfigureBluetoothEncode(SBSerialisedMessage *msg, unsigned char handle, const SBConfigureBluetoothMessage* cfg, int setRequest);
int sbConfigureBluetoothDecode(const SBSerialisedMessage *msg, SBConfigureBluetoothMessage* cfg);


// REPLY MESSAGES:
//


typedef struct {
	unsigned char status; // See SB_REPLY_...
} SBBasicReplyMessage;
int sbBasicReplyEncode(SBSerialisedMessage *msg, unsigned char handle, unsigned char msgid, const SBBasicReplyMessage* state);
#ifndef PIC30
int sbBasicReplyDecode(const SBSerialisedMessage *msg, SBBasicReplyMessage* state);
#endif

typedef struct {
	unsigned short content[2]; // same semantic has in RequestState
} SBSensorListMessage;
int sbSensorListEncode(SBSerialisedMessage *msg, unsigned char handle, const SBSensorListMessage* state);
#ifndef PIC30
int sbSensorListDecode(const SBSerialisedMessage *msg, SBSensorListMessage* state);
#endif

// SBHeliStateRaw in <sbstate.h>
#include "sbstate.h"

#ifdef SBC_HAS_IO

#include <stdio.h>

int sbSaveMessage(SBSerialisedMessage *mh, FILE * fp);

int sbLoadMessage(SBSerialisedMessage *mh, FILE * fp);

void sbPrintMessageHead(FILE *fp, SBSerialisedMessage *sm);

void sbCfgOAPrint(FILE *fp, SBConfigureObstAvoid* state);
void sbCfgCommPrint(FILE *fp, SBConfigureCommunication* state);
void sbCfgControlPrint(FILE *fp, SBConfigureControl* state);
void sbCfgTimeoutPrint(FILE *fp, SBConfigureTimeout* state);
void sbReqStatePrint(FILE *fp, SBRequestState* state);
void sbSetNavPrint(FILE *fp, SBSetNavigationMode* state);
void sbSetControlPrint(FILE *fp, SBSetControl* state);
void sbRawControlPrint(FILE *fp, SBRawControl* state);
void sbTrimModePrint(FILE *fp, SBTrimModeMessage* state);
void sbCtrlParametersPrint(FILE *fp, SBControlParametersMessage* state);

void sbBasicReplyPrint(FILE *fp, SBBasicReplyMessage* state);
void sbSensorListPrint(FILE *fp, SBSensorListMessage* state);
#endif // SBC_HAS_IO

#ifdef __cplusplus
}
#endif

#endif // SKYBOTICS_COM_H
