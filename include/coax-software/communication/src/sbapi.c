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
#include <string.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef WIN32
#define strncpy strncpy_s
#define strcasecmp _strcmpi

double round(double nr) {
   double f = floor(nr);
   double c = ceil(nr);
   return (((c-nr) >= (nr-f)) ? f :c);
}


#endif

#ifdef LINUX
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#endif

#ifdef _MSC_VER
#define inline __inline
#endif

#include "com/sbapi.h"
#include "com/sbmessage.h"
#include "com/sbstate.h"

#define D2R(X) ((X)*M_PI/180.0)
#define R2D(X) ((X)*180.0/M_PI)
#define MAX(X,Y) (((X)<(Y))?(Y):(X))
#define MIN(X,Y) (((X)<(Y))?(X):(Y))
#define DEFAULT_TIMEOUT_MS 500

#ifdef WIN32
// Open a serial communication channel to the helicopter. 
// Under linux, device may be (for instance) "/dev/tty0" or "/dev/rfcomm0" for
// bluetooth, and baudrate is a define symbol as defined in bit/termios.h (e.g.
// B115200).
// TODO: implement and test under windows/osx
int sbOpenBluetoothCommunication(struct SBControlContext *control,
		const char *port, unsigned int speed)
{
	int res;
	memset(control, 0, sizeof(struct SBControlContext));
	res = sbChannelCreateBTWin(&control->channel,port,speed);
	if (res) {
		return res;
	}
	res = sbChannelOpen(&control->channel);

	// by default, request acknowledgement
	control->ackMode = SB_MSGID_REQACK;
	
	return res;
}

// Open a network communication channel to the helicopter.
// This may be used in simulation or possibly to communicate via an on-board
// network-enabled processor. 
int sbOpenWinsockCommunication(struct SBControlContext *control,
		const char *hostname, unsigned int port)
{
	int res;
	memset(control, 0, sizeof(struct SBControlContext));
	res = sbChannelCreateWinsockUDP(&control->channel,hostname,port);
	if (res) {
		return res;
	}
	res = sbChannelOpen(&control->channel);

	// by default, request acknowledgement
	control->ackMode = SB_MSGID_REQACK;
	
	return res;
}

#else

#ifndef MACOSX
// Open a serial communication channel to the helicopter. 
// Under linux, device may be (for instance) "/dev/tty0" or "/dev/rfcomm0" for
// bluetooth, and baudrate is a define symbol as defined in bit/termios.h (e.g.
// B115200).
// TODO: implement and test under windows/osx
int sbOpenSerialCommunication(struct SBControlContext *control,
		const char *device, unsigned int baudrate, int rtscts)
{
	int res;
	bzero(control, sizeof(struct SBControlContext));
	res = sbChannelCreateSerial(&control->channel,device,baudrate,rtscts);
	if (res) {
		return res;
	}
	res = sbChannelOpen(&control->channel);

	// by default, request acknowledgement
	control->ackMode = SB_MSGID_REQACK;
	
	return res;
}
#endif

// Open a network communication channel to the helicopter.
// This may be used in simulation or possibly to communicate via an on-board
// network-enabled processor. 
int sbOpenSocketCommunication(struct SBControlContext *control,
		const char *hostname, unsigned int port)
{
	int res;
	bzero(control, sizeof(struct SBControlContext));
	res = sbChannelCreateSocketUDP(&control->channel,hostname,port);
	if (res) {
		return res;
	}
	res = sbChannelOpen(&control->channel);

	// by default, request acknowledgement
	control->ackMode = SB_MSGID_REQACK;
	
	return res;
}

#endif

// Close the communication channel and release any resource if needed
int sbCloseCommunication(struct SBControlContext *control)
{
	return sbChannelDestroy(&control->channel);
}

// Flush the communication channel 
int sbFlushCommunication(struct SBControlContext *control)
{
	return sbChannelFlush(&control->channel);
}

// Lock the communication channel to prevent concurrent access from different
// threads. Currently, this will only work on POSIX systems.
int sbLockCommunication(struct SBControlContext *control)
{
	return sbChannelLock(&control->channel);
}

// Unlock the communication channel
int sbUnlockCommunication(struct SBControlContext *control)
{
	return sbChannelUnlock(&control->channel);
}

static
void scaleHeliState(SBHeliState *hs, const SBHeliStateRaw *hr)
{
	int i;
	hs->errorFlags = hr->errorFlags;
	hs->content = ((unsigned long)hr->content[1]) |
		(((unsigned long)hr->content[0]) << 16);
	hs->timeStamp = hr->timeStamp;
	hs->controlTimeout = hr->controlTimeout;
	hs->watchdogTimeout = hr->watchdogTimeout;
	hs->mode.navigation = hr->mode.navigation;
	hs->mode.communication = hr->mode.communication;
	hs->mode.oavoid = hr->mode.oavoid;
	hs->mode.rollAxis = hr->mode.rollAxis;
	hs->mode.pitchAxis = hr->mode.pitchAxis;
	hs->mode.yawAxis = hr->mode.yawAxis;
	hs->mode.altAxis = hr->mode.altAxis;
	hs->roll = (float)( ((signed short)hr->roll) * M_PI/1800.0 );
	hs->pitch = (float)( ((signed short)hr->pitch) * M_PI/1800.0 );
	hs->yaw = (float)( ((signed short)hr->yaw) * M_PI/1800.0 );
	for (i=0;i<3;i++) {
		hs->gyro[i] = (float)( ((signed short)hr->gyro[i])*M_PI/1800.0 );
		hs->accel[i] = (float)( ((signed short)hr->accel[i])/1000.0 );
		hs->magneto[i] = (float)( ((signed short)hr->magneto[i])/1000.0 );
		hs->o_attitude[i] = (float)( ((signed short)hr->o_attitude[i])/1000.0 );
	}
	for (i=0;i<4;i++) {
		hs->hranges[i] = (float)( ((unsigned short)hr->hranges[i])/1000.0 );
	}
	hs->imutemp = (float)( ((unsigned short)hr->imutemp)/1000.0 );
	hs->battery = (float)( ((unsigned short)hr->battery)/1000.0 );
	hs->zrange = (float)( ((signed short)hr->zrange)/1000.0 );
	hs->zfiltered = (float)( ((signed short)hr->zfiltered)/1000.0 );
	hs->pressure = (float)( ((signed short)hr->pressure));
	hs->xrel = (float)( ((signed short)hr->xrel)/1000.0 );
	hs->yrel = (float)( ((signed short)hr->yrel)/1000.0 );
	hs->o_altitude = (float)( ((signed short)hr->o_altitude)/1000.0 );
	hs->o_tol = hr->o_tol;
	for (i=0;i<2;i++) {
		hs->o_xy[i] = hr->o_xy[i];
		hs->o_oavoid[i] = hr->o_oavoid[i];
	}
	for (i=0;i<8;i++) {
		hs->rcChannel[i] = (float)(hr->rcChannel[i]/1000.);
	}
    hs->coaxspeed.state = hr->coaxspeed.state;
    hs->coaxspeed.light = hr->coaxspeed.light;
    hs->coaxspeed.vel_x = (float)(hr->coaxspeed.vel_x/1000.0);
    hs->coaxspeed.vel_y = (float)(hr->coaxspeed.vel_y/1000.0);
}

// #define DEBUG_SEND_AND_WAIT
static 
int sendAndWaitMsg(struct SBControlContext *control, 
		SBSerialisedMessage *tosend,
		SBSerialisedMessage *reply)
{
	unsigned int numtries = 10;
#ifdef DEBUG_SEND_AND_WAIT
	printf("sendAndWaitMsg %02X\n",tosend->msgid);
#endif // DEBUG_SEND_AND_WAIT
	while (numtries > 0) {
		numtries --;
#ifdef DEBUG_SEND_AND_WAIT
		printf("Sending message %02X\n",tosend->msgid);
#endif // DEBUG_SEND_AND_WAIT
		if(sbSendMessage(&control->channel,tosend,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
#ifdef DEBUG_SEND_AND_WAIT
		printf("Waiting message %02X\n",tosend->msgid);
#endif // DEBUG_SEND_AND_WAIT
		if (sbWaitRawMessage(&control->channel,-1,reply,DEFAULT_TIMEOUT_MS)) {
			printf("Timeout %d\n",numtries);
			continue;
		}
#ifdef DEBUG_SEND_AND_WAIT
		printf("Got message %02X\n",reply->msgid);
#endif // DEBUG_SEND_AND_WAIT
		if (reply->msgid == (SB_MSGID_REPLY|SB_MSGID_STATE)) {
#ifdef DEBUG_SEND_AND_WAIT
			printf("That's a state message\n");
#endif
			numtries ++; // that does not count :)
			if (control->stateCallback) {
				SBHeliStateRaw hr;
				SBHeliState hs;
				sbStateDecode(reply,&hr);
				scaleHeliState(&hs,&hr);
				control->stateCallback(&hs,control->userData);
			}
		} else if (reply->msgid == (SB_MSGID_REPLY|(tosend->msgid&SB_MSGID_MASK))) {
#ifdef DEBUG_SEND_AND_WAIT
			printf("Got the ack\n");
#endif // DEBUG_SEND_AND_WAIT
			break;
		}
	}
	return numtries?0:-2;
}


// TODO: Implementation TBD
int sbConnect(struct SBControlContext *control)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBBasicReplyMessage br;
	sm.len = 0;
	sm.handle = 0;
	sm.msgid = SB_MSGID_CONNECT;
	if (control->ackMode) {
		sm.msgid |= SB_MSGID_REQACK;
	}
	sbFinalizeMessage(&sm);
	
	if (sendAndWaitMsg(control,&sm,&reply)) {
		return -1;
	}
	r = sbBasicReplyDecode(&reply,&br);
	if (r == 0) {
		control->handle = reply.handle;
		return br.status;
	} 
	return -1;
}

// TODO: Implementation TBD
int sbDisconnect(struct SBControlContext *control)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBBasicReplyMessage br;
	sm.len = 0;
	sm.handle = control->handle;
	sm.msgid = SB_MSGID_DISCONNECT;
	if (control->ackMode) {
		sm.msgid |= SB_MSGID_REQACK;
	}
	sbFinalizeMessage(&sm);
	
	control->handle = 0;
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		} 
	}else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

// This function illustrates the spirit of this API.
// At any one time, only one process/thread should use the control
// channel. 
// In synchronous mode, the dialogue is only question/response
// When the communication is continuous, the helicopter may send state
// information at any time. When a question/response mode is started, some
// state information may arrive in-between. This callback can be setup to
// handle this piece of data in this case, instead of discarding it.
// Example:
//
// // Global variable to store the state, for illustration
// SBHeliState state;
//
// void * receive_thread(void *) {
//	 ...
//	 while (1) {
//		sbLockCommunication(&control);
//		sbReceiveState(control,&state);
//		sbUnlockCommunication(&control);
//	 }
//	}
//
//	void recCB(SBHeliState *rstate, void *user_data) {
//		memcpy(&state,&rstate,sizeof(SBHeliState);
//	}
//
//	int main() {
//		struct SBControlContext control;
//		unsigned long sensors = SBS_ALL;
//		unsigned long content = SBS_ALL;
//		sbOpenSerialCommunication(&control,"/dev/rfcomm0",B115200)
//		sbConnect(&control);
//		sbRegisterStateCallback(&control,recCB,NULL);
//		sbGetSensorList(&control, &sensors);
//		sbConfigureOAMode(&control,SB_OA_NONE);
//		sbConfigureControl(&control,SB_CTRL_NONE,SB_CTRL_NONE,
//				SB_CTRL_POS, SB_CTRL_POS, SB_CTRL_VEL);
//		// In order to request only the data available on this platform
//		sbConfigureComm(SB_COM_CONTINUOUS,30,content & sensors);
//		start receive_thread()
//		while (1) {
//			wait something
//			sbLockCommunication(&control);
//			// Any state received here will be handle by the callback
//			sbSetNavMode(&control,SB_NAV_HOVER);
//			sbUnlockCommunication(&control);
//		}
//	}
int sbRegisterStateCallback(struct SBControlContext *control, 
	void (*stateCallback)(SBHeliState *state, void *userData),
	void *userData)
{
	control->stateCallback = stateCallback;
	control->userData = userData;
	return 0;
}

// Configure the communication so that control requests (sbSetNavMode,
// sbKeepAlive, sbSetControl) requires an acknowledgement
// from the helicopter. If requestAck is zero, all control functions 
// will not confirm reception of their commands. With bluetooth communication,
// this can improve the communication rate. If requestAck is non zero, the
// control request will be followed by an acknowledgement.
int sbConfigureAckMode(struct SBControlContext *control, 
		unsigned int requestAck)
{
	control->ackMode = requestAck;
	return 0;
}



int sbGetVersionInformation(struct SBControlContext *control, SBVersionStatus * status)
{
	int r;
	SBSerialisedMessage sm,reply;
	sm.len = 0;
	sm.handle = control->handle;
	sm.msgid = SB_MSGID_GET_VERSION;
	sbFinalizeMessage(&sm);
	
	if (sendAndWaitMsg(control,&sm,&reply)) {
		printf("sbGetVersionInformation: Send and wait failed\n");
		return -1;
	}
	r = sbVersionMsgDecode(&reply,status);
	if (r ==0) {
		return 0;
	} else {
		printf("sbGetVersionInformation: Decode failed\n");
	}
	return r;
}


int sbGetSensorList(struct SBControlContext *control, unsigned long *list)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBSensorListMessage sl;
	sm.len = 0;
	sm.handle = control->handle;
	sm.msgid = SB_MSGID_SENSORLIST;
	sbFinalizeMessage(&sm);
	
	if (sendAndWaitMsg(control,&sm,&reply)) {
		printf("sbGetSensorList: Send and wait failed\n");
		return -1;
	}
	r = sbSensorListDecode(&reply,&sl);
	if (r ==0) {
		*list = (sl.content[1] | (((unsigned long)sl.content[0]) << 16));
	} else {
		printf("sbGetSensorList: Decode failed\n");
	}
	return r;
}

int sbConfigureCommLoop(struct SBControlContext *control, unsigned int verbose,
		int debug_channel)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBConfigureCommLoop cl;
	SBBasicReplyMessage br;
	cl.verbosity = verbose;
	cl.debug_channel = debug_channel;
	sbCfgCommLoopEncode(&sm,control->handle,&cl,control->ackMode);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	}else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

// oavoidMode defined from sbconst.h SB_OA_xxxx
int sbConfigureOAMode(struct SBControlContext *control, unsigned int oavoidMode)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBConfigureObstAvoid oa;
	SBBasicReplyMessage br;
	oa.oavoidMode = oavoidMode;
	sbCfgOAEncode(&sm,control->handle,&oa,control->ackMode);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	}else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

// commMode defined from sbconst.h SB_COM_xxxx
int sbConfigureComm(struct SBControlContext *control, 
		unsigned int commMode,
		unsigned int frequency,
		unsigned int numMessages,
		unsigned long contents)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBConfigureCommunication comm;
	SBBasicReplyMessage br;
	comm.commMode = commMode;
	comm.frequency = frequency;
	comm.numMessages = numMessages;
	comm.content[1] = contents & 0xFFFF;
	comm.content[0] = (contents >> 16) & 0xFFFF;
	sbCfgCommEncode(&sm,control->handle,&comm,control->ackMode);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	}else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

// controlMode defined from sbconst.h SB_CTRL_xxxx
int sbConfigureControl(struct SBControlContext *control, 
		unsigned int roll,
		unsigned int pitch,
		unsigned int yaw,
		unsigned int altitude)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBConfigureControl ctrl;
	SBBasicReplyMessage br;
	ctrl.roll = roll;
	ctrl.pitch = pitch;
	ctrl.yaw = yaw;
	ctrl.altitude = altitude;
	sbCfgControlEncode(&sm,control->handle,&ctrl,control->ackMode);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	}else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

int sbConfigureTimeout(struct SBControlContext *control, 
		unsigned short control_timeout_ms,
		unsigned short watchdog_timeout_ms)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBConfigureTimeout timeout;
	SBBasicReplyMessage br;
	timeout.controlTimeout = control_timeout_ms;
	timeout.watchdogTimeout = watchdog_timeout_ms;
	sbCfgTimeoutEncode(&sm,control->handle,&timeout,control->ackMode);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	}else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

int sbKeepAlive(struct SBControlContext *control)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBBasicReplyMessage br;
	sm.len = 0;
	sm.handle = control->handle;
	sm.msgid = SB_MSGID_KEEP_ALIVE;
	if (control->ackMode) {
		sm.msgid |= SB_MSGID_REQACK;
	}
	sbFinalizeMessage(&sm);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		} 
	}else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}


/**   Control the light of the coax speed, if used as a sensor without control 
 *   */
int sbSetCoaxSpeedLight(struct SBControlContext *control, unsigned char percent)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBCoaxSpeedSetLight setlight;
	SBBasicReplyMessage br;
	setlight.percent = percent;
	sbCoaxSpeedSetLightEncode(&sm,control->handle,&setlight,control->ackMode);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	}else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

/*   Send a reset command to the DSPIC. After this command, the connections may
 *   not be reusable. 
 *   */
int sbResetDSPIC(struct SBControlContext *control)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBBasicReplyMessage br;
	sm.len = 0;
	sm.handle = control->handle;
	sm.msgid = SB_MSGID_RESET;
	if (control->ackMode) {
		sm.msgid |= SB_MSGID_REQACK;
	}
	sbFinalizeMessage(&sm);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		} 
	}else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

/*   Send a string to the DSPIC. If the debug interface are active, then this
 *   string is displayed on the DEBUG console. Otherwise it is ignored.
 *   The string length cannot be longer than SB_STRING_MESSAGE_LENGTH
 *   */
int sbSendString(struct SBControlContext *control,const char *text)
{
	SBSerialisedMessage sm;
	SBStringMessage msg;
#ifdef WIN32
	strncpy_s((char*)msg.text,SB_STRING_MESSAGE_LENGTH-1,text,SB_STRING_MESSAGE_LENGTH-1);
#else
	strncpy((char*)msg.text,text,SB_STRING_MESSAGE_LENGTH-1);
#endif
	msg.text[SB_STRING_MESSAGE_LENGTH-1]=0;
	sbStringMsgEncode(&sm,control->handle,&msg);
	
	if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
		return -1;
	}
	return 0;
}

/** Store a custom message on the dsPIC, to be read by the next
 *  sbGetCustomMessage call.
 *  The data length cannot be longer than SB_STRING_MESSAGE_LENGTH
 *   */
int sbSetCustomMessage(struct SBControlContext *control,const char message[SB_STRING_MESSAGE_LENGTH])
{
    int r=0;
	SBSerialisedMessage sm,reply;
	SBCustomMessage msg;
	SBBasicReplyMessage br;
#ifdef WIN32
	strncpy_s((char*)msg.text,SB_STRING_MESSAGE_LENGTH-1,message,SB_STRING_MESSAGE_LENGTH-1);
#else
	strncpy((char*)msg.text,message,SB_STRING_MESSAGE_LENGTH-1);
#endif
	msg.text[SB_STRING_MESSAGE_LENGTH-1]=0;
	sbCustomMsgEncode(&sm,control->handle,&msg,0);
	
	if (sendAndWaitMsg(control,&sm,&reply)) {
		printf("sSGetCustomMessage: Send and wait failed\n");
		return -1;
	}
    r = sbBasicReplyDecode(&reply,&br);
    if (r == 0) {
        return br.status;
    }
	return -2;
}

/** Read the last custom message from the dsPIC.*/
int sbGetCustomMessage(struct SBControlContext *control,char message[SB_STRING_MESSAGE_LENGTH])
{
	int r;
	SBSerialisedMessage sm,reply;
	SBCustomMessage msg;
	sm.len = 0;
	sm.handle = control->handle;
	sm.msgid = SB_MSGID_CUSTOM;
	sbFinalizeMessage(&sm);
	
	if (sendAndWaitMsg(control,&sm,&reply)) {
		printf("sbGetCustomMessage: Send and wait failed\n");
		return -1;
	}
	r = sbCustomMsgDecode(&reply,&msg);
	if (r ==0) {
#ifdef WIN32
        strncpy_s((char*)message,SB_STRING_MESSAGE_LENGTH-1,(char*)msg.text,SB_STRING_MESSAGE_LENGTH-1);
#else
        strncpy((char*)message,(char*)msg.text,SB_STRING_MESSAGE_LENGTH-1);
#endif
        message[SB_STRING_MESSAGE_LENGTH-1]=0;
	} else {
		printf("sbGetCustomMessage: Decode failed\n");
	}
	return r;
}

// Use a OR of SBS_... flags to create content
int sbRequestState(struct SBControlContext *control,
		unsigned long contents, SBHeliState *state)
{
	unsigned int numtry = 0;
	SBSerialisedMessage sm;
	SBRequestState rs;
	rs.content[1] = contents & 0xFFFF;
	rs.content[0] = (contents >> 16) & 0xFFFF;
	sbReqStateEncode(&sm,control->handle,&rs);
	// dumpMessage("sbReqStateEncode",&sm);

	do {
		numtry += 1;
		if (numtry > 5) {
			printf("sbRequestState: timeout\n");
			return -3;
		}
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			printf("sbRequestState: sbSendMessage failed\n");
			state->content = 0;
		}
		if(sbReceiveState(control,state,DEFAULT_TIMEOUT_MS)) {
			printf("sbRequestState: sbReceiveState failed\n");
			state->content = 0;
		}
#ifdef LINUX
		usleep(10000);
#endif
#ifdef WIN32
		Sleep(10);
#endif
		// If the state content is not what we asked for, request it again!
	} while (state->content != contents);
	return 0;
}

int sbWaitState(struct SBControlContext *control, unsigned int timeout_ms)
{
	return sbChannelWaitData(&control->channel, timeout_ms);
}

// Content controlled via sbConfigureComm
int sbReceiveState(struct SBControlContext *control,SBHeliState *state, 
		unsigned int timeout_ms)
{
	SBSerialisedMessage reply;
	SBHeliStateRaw hr;
	if (sbWaitRawMessage(&control->channel,-1,&reply,timeout_ms)) {
		return -1;
	}
	// sbDumpMessage("sbReceiveState",&reply);
	if (reply.msgid == (SB_MSGID_REPLY|SB_MSGID_STATE)) {
		sbStateDecode(&reply,&hr);
		scaleHeliState(state,&hr);
		return 0;
	}
	return -2;
}

// navMode defined from sbconst.h SB_NAV_xxxx
int sbSetNavMode(struct SBControlContext *control, 
		unsigned int navMode)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBSetNavigationMode nav;
	SBBasicReplyMessage br;
	nav.mode = navMode;
	sbSetNavEncode(&sm,control->handle,&nav,control->ackMode);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	} else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
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
#ifndef EPOCHFILETIME
#ifndef __GNUC__
#define EPOCHFILETIME (116444736000000000i64)
#else
#define EPOCHFILETIME (116444736000000000LL)
#endif
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

double sbGetCurrentTime() {
	return now();
}

//#define DEBUG(c) {int __res__;printf("Executing "#c"\n");__res__ = (c);printf("Result %d\n",__res__);}
#define DEBUG(c) {int __res__ = (c);if (__res__) printf("Result of "#c": %d\n",__res__);}
int sbSetNavAndWait(struct SBControlContext *control, 
		int setmode, int waitmode,double timout_sec)
{
	SBHeliState state;
	int res;
	double t,t0 = now();
	sbLockCommunication(control);
	DEBUG(res = sbSetNavMode(control,setmode));
	sbUnlockCommunication(control);
	if (res) return res;
	while (1) {
		sbLockCommunication(control);
		DEBUG(res = sbRequestState(control, SBS_MODES, &state));
		sbUnlockCommunication(control);
		if (res) return res;

		if (state.mode.navigation == waitmode){
			break;
		}
		t = now(); 
		if (t < t0) t += 86400; // time of day wrap around
		if ((t - t0) > timout_sec) {
			break;
		}
#ifdef LINUX
		usleep(5000);
#endif
#ifdef WIN32
		Sleep(5);
#endif
	}
	return (state.mode.navigation == waitmode)?0:+1;
}


// Only SI unit used here: percent, percent, rad/s, m, m/s
int sbSetControl(struct SBControlContext *control, 
		double roll, double pitch, double yaw, double altitude)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBSetControl ctrl;
	SBBasicReplyMessage br;
	ctrl.roll = (signed short)round(roll*1000);
	ctrl.pitch = (signed short)round(pitch*1000);
	ctrl.yaw = (signed short)round(yaw*1800.0/M_PI);
	ctrl.altitude = (signed short)round(altitude*1000.0);
	sbSetControlEncode(&sm,control->handle,&ctrl,control->ackMode);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	} else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

// Only SI unit used here: percent, percent, rad/s, m, m/s
int sbSetControlWithTimestamp(struct SBControlContext *control, 
		double roll, double pitch, double yaw, double altitude, unsigned long timestamp)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBSetControlWithTimestamp ctrl;
	SBBasicReplyMessage br;
	ctrl.roll = (signed short)round(roll*1000);
	ctrl.pitch = (signed short)round(pitch*1000);
	ctrl.yaw = (signed short)round(yaw*1800.0/M_PI);
	ctrl.altitude = (signed short)round(altitude*1000.0);
	ctrl.timestamp = timestamp;
	sbSetControlWithTimestampEncode(&sm,control->handle,&ctrl,control->ackMode);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	} else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}


int sbConfigureRawControl(struct SBControlContext *control, 
		unsigned char speedprofile1, unsigned char speedprofile2)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBConfigureRawControl cfg;
	SBBasicReplyMessage br;
	cfg.speedprofile1 = speedprofile1;
	cfg.speedprofile2 = speedprofile2;
	sbCfgRawControlEncode(&sm,control->handle,&cfg,control->ackMode);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	} else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

int sbSetRawControl(struct SBControlContext *control, 
		double motor1, double motor2, double servo1, double servo2)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBRawControl ctrl;
	SBBasicReplyMessage br;
	ctrl.motor1 = (signed short)(motor1*1000);
	ctrl.motor2 = (signed short)(motor2*1000);
	ctrl.servo1 = (signed short)(servo1*1000);
	ctrl.servo2 = (signed short)(servo2*1000);
	sbRawControlEncode(&sm,control->handle,&ctrl,control->ackMode);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	} else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

int sbGetTrimMode(struct SBControlContext *control, struct SBTrimMode *mode)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBTrimModeMessage tm;
	sm.len = 0;
	sm.handle = control->handle;
	sm.msgid = SB_MSGID_TRIMMODE | (control->ackMode?SB_MSGID_REQACK:0);
	sbFinalizeMessage(&sm);
	
	if (sendAndWaitMsg(control,&sm,&reply)) {
		return -1;
	}
	r = sbTrimModeDecode(&reply,&tm);
	if (r ==0) {
		mode->trimMode = tm.trimMode;
		mode->rollTrim = (float)(tm.rollTrim/10000.);
		mode->pitchTrim = (float)(tm.pitchTrim/10000.);
	}
	return r;
}

int sbSetTrimMode(struct SBControlContext *control, const struct SBTrimMode *mode)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBTrimModeMessage tm;
	SBBasicReplyMessage br;
	tm.trimMode = mode->trimMode;
	tm.rollTrim = (signed short)(10000 * mode->rollTrim);
	tm.pitchTrim = (signed short)(10000 * mode->pitchTrim);
	sbTrimModeEncode(&sm,control->handle,&tm,1);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	} else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}


int sbGetControlParameters(struct SBControlContext *control, 
		struct SBControlParameters *params)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBControlParametersMessage cm;
	sm.len = 0;
	sm.handle = control->handle;
	sm.msgid = SB_MSGID_CTRLPARM | (control->ackMode?SB_MSGID_REQACK:0);
	sbFinalizeMessage(&sm);
	
	if (sendAndWaitMsg(control,&sm,&reply)) {
		return -1;
	}
	r = sbCtrlParametersDecode(&reply,&cm);
	if (r ==0) {
		params->baseThrust = (float)(cm.baseThrust/1000.);
		params->yawOffset = (float)(cm.yawOffset/1000.);
		params->altitudeKp = (float)(cm.altitudeKp/1000.);
		params->altitudeKi = (float)(cm.altitudeKi/1000.);
		params->altitudeKd = (float)(cm.altitudeKd/1000.);
		params->yawKp = (float)(cm.yawKp/1000.);
		params->yawKi = (float)(cm.yawKi/1000.);
		params->yawKd = (float)(cm.yawKd/1000.);
	}
	return r;
}

int sbSetControlParameters(struct SBControlContext *control, 
		const struct SBControlParameters *params)
{
	int r;
	SBSerialisedMessage sm,reply;
	SBControlParametersMessage cm;
	SBBasicReplyMessage br;
	cm.baseThrust = (unsigned short)(1000 * params->baseThrust);
	cm.yawOffset = (unsigned short)(1000 * params->yawOffset);
	cm.altitudeKp = (signed short)(1000 * params->altitudeKp);
	cm.altitudeKi = (signed short)(1000 * params->altitudeKi);
	cm.altitudeKd = (signed short)(1000 * params->altitudeKd);
	cm.yawKp = (signed short)(1000 * params->yawKp);
	cm.yawKi = (signed short)(1000 * params->yawKi);
	cm.yawKd = (signed short)(1000 * params->yawKd);
	sbCtrlParametersEncode(&sm,control->handle,&cm,1);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	} else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

int sbConfigureBluetooth(struct SBControlContext *control,
		const char code[4], const char name[16])
{
	int r;
	SBSerialisedMessage sm,reply;
	SBConfigureBluetoothMessage cm;
	SBBasicReplyMessage br;
	memcpy(cm.code,code,4);
	memcpy(cm.name,name,16);
	sbConfigureBluetoothEncode(&sm,control->handle,&cm,1);
	
	if (control->ackMode) {
		if (sendAndWaitMsg(control,&sm,&reply)) {
			return -1;
		}
		r = sbBasicReplyDecode(&reply,&br);
		if (r == 0) {
			return br.status;
		}
	} else {
		if(sbSendMessage(&control->channel,&sm,DEFAULT_TIMEOUT_MS)) {
			return -1;
		}
		return 0;
	}
	return -1;
}

#ifdef SBC_HAS_IO

#define TESTCONTENT(content,what) ((content) & (what))

const char* sbContentString(unsigned long c)
{
	switch (c) {
		case SBS_MODES:
			return "MODES";
		case SBS_TIMESTAMP:
			return "TIMESTAMP";
		case SBS_RPY:
			return "RPY";
		case SBS_GYRO:
			return "GYRO";
		case SBS_ACCEL:
			return "ACCEL";
		case SBS_MAGNETO:
			return "MAGNETO";
		case SBS_IMUTEMP:
			return "IMUTEMP";
		case SBS_ALTITUDE:
			return "ALTITUDE";
		case SBS_PRESSURE:
			return "PRESSURE";
		case SBS_HRANGES:
			return "HRANGES";
		case SBS_XY_REL:
			return "XY_REL";
		case SBS_BATTERY:
			return "BATTERY";
		case SBS_CHANNELS:
			return "CHANNELS";
		case SBS_O_ATTITUDE:
			return "O_ATTITUDE";
		case SBS_O_ALTITUDE:
			return "O_ALTITUDE";
		case SBS_O_TOL:
			return "O_TOL";
		case SBS_O_XY:
			return "O_XY";
		case SBS_O_OAVOID:
			return "O_OAVOID";
		case SBS_TIMEOUT:
			return "TIMEOUT";
		case SBS_COAXSPEED:
			return "COAXSPEED";
		default:
			break;
	}
	return NULL;
}

const char * sbFlagString(unsigned char err) {
    switch (err) {
        case SB_FLAG_IMUCRASH: return "IMUCRASH";
        case SB_FLAG_LOWPOWER: return "LOWPOWER";
        case SB_FLAG_RCLOST: return "RCLOST";
        case SB_FLAG_KILLED: return "KILLED";
        case SB_FLAG_MANUAL: return "MANUAL";
        default : return "UNKNOWN";
    }
    return NULL;
}

void sbErrorPrint(FILE * fp, unsigned char err) {
	fprintf(fp,"Error  : %02X [ ",err);
    if (err & SB_FLAG_IMUCRASH) fprintf(fp,"%s ",sbFlagString(SB_FLAG_IMUCRASH));
    if (err & SB_FLAG_LOWPOWER) fprintf(fp,"%s ",sbFlagString(SB_FLAG_LOWPOWER));
    if (err & SB_FLAG_RCLOST) fprintf(fp,"%s ",sbFlagString(SB_FLAG_RCLOST));
    if (err & SB_FLAG_MANUAL) fprintf(fp,"%s ",sbFlagString(SB_FLAG_MANUAL));
    if (err & SB_FLAG_KILLED) fprintf(fp,"%s ",sbFlagString(SB_FLAG_KILLED));
    fprintf(fp,"]\n");
}

void sbStatePrint(FILE * fp, const SBHeliState *hs) 
{
	fprintf(fp,"Content: "); sbContentPrint(fp,hs->content);
	fprintf(fp,"\n");
    sbErrorPrint(fp,hs->errorFlags);
	if (TESTCONTENT(hs->content,SBS_MODES)) {
		fprintf(fp,"Modes:\n");
		fprintf(fp,"\tNavigation %s",sbNavModeString(hs->mode.navigation));
		fprintf(fp,"\tCommunication %s",sbCommModeString(hs->mode.communication));
		fprintf(fp,"\tObst. Avoid: %s\n",sbOAModeString(hs->mode.oavoid));
		fprintf(fp,"\tAxis: Roll %s",sbCtrlModeString(hs->mode.rollAxis));
		fprintf(fp,"\tPitch %s",sbCtrlModeString(hs->mode.pitchAxis));
		fprintf(fp,"\tYaw %s",sbCtrlModeString(hs->mode.yawAxis));
		fprintf(fp,"\tAlti. %s",sbCtrlModeString(hs->mode.altAxis));
		fprintf(fp,"\n");
	}
	if (TESTCONTENT(hs->content,SBS_TIMESTAMP)) {
		fprintf(fp,"Timestamp: %8ld\n",hs->timeStamp);
	}
	if (TESTCONTENT(hs->content,SBS_RPY)) {
		fprintf(fp,"RPY (deg): %+6.2f, %+6.2f, %+6.2f\n",R2D(hs->roll), R2D(hs->pitch), R2D(hs->yaw));
	}
	if (TESTCONTENT(hs->content,SBS_GYRO)) {
		fprintf(fp,"Gyro (deg/s): %+6.2f, %+6.2f, %+6.2f\n",R2D(hs->gyro[0]), R2D(hs->gyro[1]), R2D(hs->gyro[2]));
	}
	if (TESTCONTENT(hs->content,SBS_ACCEL)) {
		fprintf(fp,"Accel (m/s2): %+5.2f, %+5.2f, %+5.2f\n",hs->accel[0], hs->accel[1], hs->accel[2]);
	}
	if (TESTCONTENT(hs->content,SBS_MAGNETO)) {
		fprintf(fp,"Magneto: %+5.2f, %+5.2f, %+5.2f\n",hs->magneto[0], hs->magneto[1], hs->magneto[2]);
	}
	if (TESTCONTENT(hs->content,SBS_IMUTEMP)) {
		fprintf(fp,"IMU Temp (degC): %.2f\n",hs->imutemp);
	}
	if (TESTCONTENT(hs->content,SBS_ALTITUDE)) {
		fprintf(fp,"Z Range (m): %+4.2f Filtered (m): %.2f\n",
				hs->zrange, hs->zfiltered);
	}
	if (TESTCONTENT(hs->content,SBS_PRESSURE)) {
		fprintf(fp,"Pressure (?): %+6.2f\n",hs->pressure);
	}
	if (TESTCONTENT(hs->content,SBS_HRANGES)) {
		int i;
		fprintf(fp,"H ranges (m): ");
		for (i=0;i<4;i++) {
			if (hs->hranges[i]>65.5) {
				fprintf(fp,"inf "); 
			} else {
				fprintf(fp,"%4.2f ",hs->hranges[i]);
			}
		}
		printf("\n");
	}
	if (TESTCONTENT(hs->content,SBS_XY_REL)) {
		fprintf(fp,"XY Rel (m): %.2f, %.2f\n",hs->xrel, hs->yrel);
	}
	if (TESTCONTENT(hs->content,SBS_BATTERY)) {
		fprintf(fp,"Battery (V): %5.2f\n",hs->battery);
	}
	if (TESTCONTENT(hs->content,SBS_TIMEOUT)) {
		fprintf(fp,"Timeout (ms): WD %d CTRL %d \n",hs->watchdogTimeout,hs->controlTimeout);
	}
	if (TESTCONTENT(hs->content,SBS_CHANNELS)) {
		fprintf(fp,"Channels: %.3f %.3f %.3f %.3f\n\t%.3f %.3f %.3f %.3f \n",
				hs->rcChannel[0],hs->rcChannel[1],
				hs->rcChannel[2],hs->rcChannel[3],
				hs->rcChannel[4],hs->rcChannel[5],
				hs->rcChannel[6],hs->rcChannel[7]);
	}
	if (TESTCONTENT(hs->content,SBS_O_ATTITUDE)) {
		fprintf(fp,"Output Attitude: %f, %f, %f\n",hs->o_attitude[0], hs->o_attitude[1], hs->o_attitude[2]);
	}
	if (TESTCONTENT(hs->content,SBS_O_ALTITUDE)) {
		fprintf(fp,"Output Altitude: %f\n", hs->o_altitude);
	}
	if (TESTCONTENT(hs->content,SBS_O_TOL)) {
		fprintf(fp,"Output TOL: %04X\n",hs->o_tol);
	}
	if (TESTCONTENT(hs->content,SBS_O_XY)) {
		fprintf(fp,"Output XY: %f, %f\n",hs->o_xy[0], hs->o_xy[1]);
	}
	if (TESTCONTENT(hs->content,SBS_O_OAVOID)) {
		fprintf(fp,"Output OA XY: %f, %f\n", hs->o_oavoid[0], hs->o_oavoid[1]);
	}
	if (TESTCONTENT(hs->content,SBS_COAXSPEED)) {
        if (hs->coaxspeed.state & COAXSPEED_AVAILABLE) {
            if (hs->coaxspeed.state & COAXSPEED_VALID_MEASUREMENT) {
                fprintf(fp,"CoaxSpeed %02X: V %.3f %.3f L %3d%%\n",hs->coaxspeed.state,
                        hs->coaxspeed.vel_x, hs->coaxspeed.vel_y, hs->coaxspeed.light);
            } else {
                fprintf(fp,"CoaxSpeed %02X: V  invalid  L %3d%%\n",hs->coaxspeed.state, hs->coaxspeed.light);
            }
        } else {
            fprintf(fp,"CoaxSpeed %02X: not detected\n",hs->coaxspeed.state);
        }
	}
}

void sbContentPrint(FILE* fp, unsigned long content)
{
	fprintf(fp,"%08lX [",content);
	if (TESTCONTENT(content,SBS_MODES)) {
		fprintf(fp,"%s,",sbContentString(SBS_MODES));
	}
	if (TESTCONTENT(content,SBS_TIMESTAMP)) {
		fprintf(fp,"%s,",sbContentString(SBS_TIMESTAMP));
	}
	if (TESTCONTENT(content,SBS_RPY)) {
		fprintf(fp,"%s,",sbContentString(SBS_RPY));
	}
	if (TESTCONTENT(content,SBS_GYRO)) {
		fprintf(fp,"%s,",sbContentString(SBS_GYRO));
	}
	if (TESTCONTENT(content,SBS_ACCEL)) {
		fprintf(fp,"%s,",sbContentString(SBS_ACCEL));
	}
	if (TESTCONTENT(content,SBS_MAGNETO)) {
		fprintf(fp,"%s,",sbContentString(SBS_MAGNETO));
	}
	if (TESTCONTENT(content,SBS_IMUTEMP)) {
		fprintf(fp,"%s,",sbContentString(SBS_IMUTEMP));
	}
	if (TESTCONTENT(content,SBS_ALTITUDE)) {
		fprintf(fp,"%s,",sbContentString(SBS_ALTITUDE));
	}
	if (TESTCONTENT(content,SBS_PRESSURE)) {
		fprintf(fp,"%s,",sbContentString(SBS_PRESSURE));
	}
	if (TESTCONTENT(content,SBS_HRANGES)) {
		fprintf(fp,"%s,",sbContentString(SBS_HRANGES));
	}
	if (TESTCONTENT(content,SBS_XY_REL)) {
		fprintf(fp,"%s,",sbContentString(SBS_XY_REL));
	}
	if (TESTCONTENT(content,SBS_BATTERY)) {
		fprintf(fp,"%s,",sbContentString(SBS_BATTERY));
	}
	if (TESTCONTENT(content,SBS_TIMEOUT)) {
		fprintf(fp,"%s,",sbContentString(SBS_TIMEOUT));
	}
	if (TESTCONTENT(content,SBS_CHANNELS)) {
		fprintf(fp,"%s,",sbContentString(SBS_CHANNELS));
	}
	if (TESTCONTENT(content,SBS_O_ATTITUDE)) {
		fprintf(fp,"%s,",sbContentString(SBS_O_ATTITUDE));
	}
	if (TESTCONTENT(content,SBS_O_ALTITUDE)) {
		fprintf(fp,"%s,",sbContentString(SBS_O_ALTITUDE));
	}
	if (TESTCONTENT(content,SBS_O_TOL)) {
		fprintf(fp,"%s,",sbContentString(SBS_O_TOL));
	}
	if (TESTCONTENT(content,SBS_O_XY)) {
		fprintf(fp,"%s,",sbContentString(SBS_O_XY));
	}
	if (TESTCONTENT(content,SBS_O_OAVOID)) {
		fprintf(fp,"%s,",sbContentString(SBS_O_OAVOID));
	}
	if (TESTCONTENT(content,SBS_COAXSPEED)) {
		fprintf(fp,"%s,",sbContentString(SBS_COAXSPEED));
	}
	fprintf(fp,"]");
}

int sbPrintControlParameters(FILE * fp, const struct SBControlParameters * params)
{
	if (!fp) return -1;
	fprintf(fp,"baseThrust %e\n",params->baseThrust);
	fprintf(fp,"yawOffset %e\n",params->yawOffset);
	fprintf(fp,"altitudeKp %e\n",params->altitudeKp);
	fprintf(fp,"altitudeKi %e\n",params->altitudeKi);
	fprintf(fp,"altitudeKd %e\n",params->altitudeKd);
	fprintf(fp,"yawKp %e\n",params->yawKp);
	fprintf(fp,"yawKi %e\n",params->yawKi);
	fprintf(fp,"yawKd %e\n",params->yawKd);
	return 0;
}

int sbSaveControlParamters(const char * fname, 
		const struct SBControlParameters * params)
{
	int res;
	FILE * fp = fopen(fname,"w");
	if (!fp) {perror("sbSaveControlParamters:");return -1;}
	res = sbPrintControlParameters(fp,params);
	fclose(fp);
	return res;
}

int sbLoadControlParameters(const char * fname,
		struct SBControlParameters * params)
{
	char line[80],pname[80];
	float value;
	int res = 0;
	FILE * fp = stdin;
	if (strcmp(fname,"-")!=0) {
		fp = fopen(fname,"r");
		if (!fp) {perror("sbLoadControlParameters:");return -1;}
	}
	while (!feof(fp)) {
		line[0] = 0;
		fgets(line,79,fp);
		if (line[0] == '#') {
			continue;
		}
		if (sscanf(line," %s %e ",pname,&value) != 2) {
			continue;
		}
		if (strcasecmp(pname,"baseThrust")==0) {
			params->baseThrust = value;
		}
		if (strcasecmp(pname,"yawOffset")==0) {
			params->yawOffset = value;
		}
		if (strcasecmp(pname,"altitudeKp")==0) {
			params->altitudeKp = value;
		}
		if (strcasecmp(pname,"altitudeKd")==0) {
			params->altitudeKd = value;
		}
		if (strcasecmp(pname,"altitudeKi")==0) {
			params->altitudeKi = value;
		}
		if (strcasecmp(pname,"yawKp")==0) {
			params->yawKp = value;
		}
		if (strcasecmp(pname,"yawKi")==0) {
			params->yawKi = value;
		}
		if (strcasecmp(pname,"yawKd")==0) {
			params->yawKd = value;
		}
	}
	if (fp != stdin) {
		fclose(fp);
	}
	printf("Loaded control parameters:\n");
	sbPrintControlParameters(stdout,params);
	return res;
}



#endif

