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
#include <stdio.h>
#ifdef DEFENSIVE
#include <assert.h>
#endif


#include "com/sbmessage.h"
// #define DEBUG_MESSAGE
#ifdef WIN32
#ifdef NDEBUG
#define assert(x) 
#endif

#ifdef DEFENSIVE
#define PRINTF(X,...) printf(X,__VA_ARGS__)
#else
#define PRINTF(X,...) 
#endif

#ifdef DEBUG_MESSAGE
#define VPRINTF(X,...) printf(X,__VA_ARGS__)
#else
#define VPRINTF(X,...) 
#endif

#else
#ifdef DEFENSIVE
#define PRINTF(X...) printf(X)
#else
#define PRINTF(X...) 
#endif

#ifdef DEBUG_MESSAGE
#define VPRINTF(X...) printf(X)
#else
#define VPRINTF(X...) 
#endif
#endif

#ifdef SBC_HAS_IO
void sbDumpMessage(const char * prefix, const SBSerialisedMessage *sm);
#endif

/**
 * Compute the CRC16 polynomial on byte string.
 *
 * @param p pointer to start of byte string
 * @param n length of byte string
 * @return the CRC16 result
 */

typedef unsigned char crc;
static crc  crcTable[256] = {0x42,};

#define WIDTH  (8 * sizeof(crc))
#define TOPBIT (1 << (WIDTH - 1))
#define POLYNOMIAL 0xD8  /* 11011000 */


static void crcInit(void)
{
	unsigned short dividend,bit;
    crc  remainder;


    /*
     * Compute the remainder of each possible dividend.
     */
    for (dividend = 0; dividend < 256; ++dividend)
    {
        /*
         * Start with the dividend followed by zeros.
         */
        remainder = dividend << (WIDTH - 8);

        /*
         * Perform modulo-2 division, a bit at a time.
         */
        for (bit = 8; bit > 0; --bit)
        {
            /*
             * Try to divide the current data bit.
             */			
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }

        /*
         * Store the result into the table.
         */
        crcTable[dividend] = remainder;
    }

}   /* crcInit() */

static crc
crcFast(const unsigned char message[], unsigned int nBytes)
{
    unsigned char data,byte;
    unsigned short remainder = 0;
	if (crcTable[0] == 0x42) {
		crcInit();
	}

    /*
     * Divide the message by the polynomial, a byte at a time.
     */
    for (byte = 0; byte < nBytes; ++byte)
    {
        data = message[byte] ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);
    }

    /*
     * The final remainder is the CRC.
     */
    return (crc)(remainder);

}   /* crcFast() */


static 
unsigned char computeCRC(SBSerialisedMessage *sm)
{
#if 0
	// This implementation does not work on a 16 bit cpu (or at least does not
	// give the same result 
	unsigned int crc16, crc_data, i;

	crc16 = 0;
	crc_data = 0;
	for (i=0; i<sm->len+3; i++) {
		crc_data <<= 8;
		crc_data |= (unsigned int)sm->data[i];
		crc16 <<= 1;
		if ((crc16 & 0x10000) > 0)
			crc16 ^= 0x8005;
		crc16 ^= crc_data;
		crc16 &= 0xffff;
	}

	return (crc16);
#else
	return crcFast(sm->data,sm->len+SB_MESSAGE_HEADER_LEN);
#endif
}

int sbInitializeMessage(SBSerialisedMessage *sm, 
		unsigned char msgid, unsigned char handle)
{
#ifdef DEFENSIVE
	memset(sm,0,sizeof(SBSerialisedMessage));
#endif
	sm->msgid = msgid;
	sm->handle = handle;
	sm->len = 0;
	return 0;
}

int sbFinalizeMessage(SBSerialisedMessage *sm)
{
	sm->data[0] = SB_MESSAGE_START_FRAME;
	sm->data[1] = sm->msgid;
	sm->data[2] = sm->handle;
	sm->data[3] = sm->len;
#ifdef DEFENSIVE
	assert(sm->len+5 <= SB_MAX_MESSAGE_SIZE);
#endif
	sm->data[sm->len+SB_MESSAGE_HEADER_LEN] = sm->crc = computeCRC(sm);

	return 0;
}

#define shortofch2(i0,i8) ((unsigned short)i0 | ((unsigned short)i8 << 8)) 
#ifdef DEFENSIVE
#define CHECK_DEC(name,mid,mlen) \
	if ((msg->msgid & ~(SB_MSGID_REQACK|SB_MSGID_REPLY)) != (mid)) {           \
		PRINTF(#name": invalid msgid: %02X/%02X instead of %02X\n", \
				msg->msgid,msg->msgid & ~(SB_MSGID_REQACK|SB_MSGID_REPLY),(mid));      \
		return -1;                                     \
	}                                                  \
	if (msg->len != (mlen)) {                             \
		PRINTF(#name": invalid msg len: %d instead of %d\n",msg->len,(mlen));    \
		return -1;                                     \
	}
#else
#define CHECK_DEC(name,id,len) 
#endif


#ifndef PIC30
int sbCfgOAEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBConfigureObstAvoid* state, int requestAck)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,SB_MSGID_CFG_OAVOID | (requestAck?SB_MSGID_REQACK:0), handle);
	m[0] = state->oavoidMode;
	msg->len = 1;
	return sbFinalizeMessage(msg);
}
#endif

int sbCfgOADecode(const SBSerialisedMessage *msg, SBConfigureObstAvoid* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbCfgOADecode,SB_MSGID_CFG_OAVOID,1);
	state->oavoidMode = m[0];
	return 0;
}

#ifndef PIC30
int sbCoaxSpeedSetLightEncode(SBSerialisedMessage *msg, unsigned char handle, 
        const SBCoaxSpeedSetLight* setlight, int requestAck)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,SB_MSGID_SET_LIGHT | (requestAck?SB_MSGID_REQACK:0), handle);
	m[0] = setlight->percent;
	msg->len = 1;
	return sbFinalizeMessage(msg);
}
#endif

int sbCoaxSpeedSetLightDecode(const SBSerialisedMessage *msg, 
        SBCoaxSpeedSetLight* setlight)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbCfgOADecode,SB_MSGID_SET_LIGHT,1);
	setlight->percent = m[0];
	return 0;
}

#ifndef PIC30
int sbCfgCommLoopEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBConfigureCommLoop* state, int requestAck)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,SB_MSGID_CFG_COMMLOOP | (requestAck?SB_MSGID_REQACK:0), handle);
	m[0] = state->verbosity;
	m[1] = state->debug_channel;
	msg->len = 2;
	return sbFinalizeMessage(msg);
}
#endif

int sbCfgCommLoopDecode(const SBSerialisedMessage *msg, SBConfigureCommLoop* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbCfgCommLoopDecode,SB_MSGID_CFG_COMMLOOP,2);
	state->verbosity = m[0];
	state->debug_channel = m[1];
	return 0;
}

#define UC(X) ((unsigned char)((X)))


#ifndef PIC30
int sbCfgCommEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBConfigureCommunication* state, int requestAck)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,SB_MSGID_CFG_COMM | (requestAck?SB_MSGID_REQACK:0), handle);
	m[0] = UC(state->commMode);
	m[1] = UC(state->frequency);
	m[2] = UC(state->content[1]);
	m[3] = UC((state->content[1] >> 8));
	m[4] = UC(state->content[0]);
	m[5] = UC(state->numMessages);
	m[6] = UC(state->numMessages >> 8);
	msg->len = 7;
	return sbFinalizeMessage(msg);
}
#endif

int sbCfgCommDecode(const SBSerialisedMessage *msg, SBConfigureCommunication* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbCfgCommDecode,SB_MSGID_CFG_COMM,7);
	state->commMode = m[0];
	state->frequency = m[1];
	state->content[1] = shortofch2(m[2],m[3]);
	state->content[0] = shortofch2(m[4],0);
	state->numMessages = shortofch2(m[5],m[6]);
	return 0;
}

#ifndef PIC30
int sbCfgControlEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBConfigureControl* state, int requestAck)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,SB_MSGID_CFG_CONTROL | (requestAck?SB_MSGID_REQACK:0), handle);
	m[0] = ((state->roll & 0x0F) << 0) 
		| ((state->pitch & 0x0F) << 4);
	m[1] = ((state->yaw & 0x0F) << 0) 
		| ((state->altitude & 0x0F) << 4);
	printf("Configure control: %02X %02X\n",m[0],m[1]);
	msg->len = 2;
	return sbFinalizeMessage(msg);
}
#endif

int sbCfgControlDecode(const SBSerialisedMessage *msg, SBConfigureControl* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbCfgControlDecode,SB_MSGID_CFG_CONTROL,2);
#ifndef PIC30
	printf("Configure control: %02X %02X\n",m[0],m[1]);
#endif
	state->roll = m[0] & 0x0F;
	state->pitch = (m[0] >> 4) & 0x0F;
	state->yaw = m[1] & 0x0F;
	state->altitude = (m[1] >> 4) & 0x0F;
#ifdef SBC_HAS_IO
	sbCfgControlPrint(stderr,state);
#endif
	return 0;
}

#ifndef PIC30
int sbReqStateEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBRequestState* state)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,SB_MSGID_STATE | SB_MSGID_REQACK, handle);
	m[0] = UC(state->content[1]);
	m[1] = UC((state->content[1] >> 8));
	m[2] = UC(state->content[0]);
	msg->len = 3;
	return sbFinalizeMessage(msg);
}
#endif

int sbReqStateDecode(const SBSerialisedMessage *msg, SBRequestState* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbReqStateDecode,SB_MSGID_STATE,3);
	state->content[1] = shortofch2(m[0],m[1]);
	state->content[0] = shortofch2(m[2],0);
	return 0;
}

#ifndef PIC30
int sbSetNavEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBSetNavigationMode* state, int requestAck)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,
			SB_MSGID_SET_NAVMODE | (requestAck?SB_MSGID_REQACK:0), handle);
	m[0] = state->mode;
	msg->len = 1;
	return sbFinalizeMessage(msg);
}
#endif

int sbSetNavDecode(const SBSerialisedMessage *msg, SBSetNavigationMode* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbSetNavDecode,SB_MSGID_SET_NAVMODE,1);
	state->mode = m[0];
	return 0;
}

#ifndef PIC30
int sbSetControlWithTimestampEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBSetControlWithTimestamp* state, int requestAck)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,
			SB_MSGID_SET_CONTROL_WITH_TIMESTAMP | (requestAck?SB_MSGID_REQACK:0), handle);
	m[ 0] = UC(state->roll);
	m[ 1] = UC(state->roll >> 8);
	m[ 2] = UC(state->pitch);
	m[ 3] = UC(state->pitch >> 8);
	m[ 4] = UC(state->yaw);
	m[ 5] = UC(state->yaw >> 8);
	m[ 6] = UC(state->altitude);
	m[ 7] = UC(state->altitude >> 8);
	m[ 8] = UC(state->timestamp);
	m[ 9] = UC(state->timestamp >> 8);
	m[10] = UC(state->timestamp >>16);
	m[11] = UC(state->timestamp >>24);
	msg->len = 12;
	return sbFinalizeMessage(msg);
}
#endif

int sbSetControlWithTimestampDecode(const SBSerialisedMessage *msg, SBSetControlWithTimestamp* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbSetControlDecodeWithTimestamp,SB_MSGID_SET_CONTROL_WITH_TIMESTAMP,12);
	state->roll = shortofch2(m[0],m[1]);
	state->pitch = shortofch2(m[2],m[3]);
	state->yaw = shortofch2(m[4],m[5]);
	state->altitude = shortofch2(m[6],m[7]);
	state->timestamp = m[11];
	state->timestamp = (state->timestamp << 8) | m[10];
	state->timestamp = (state->timestamp << 8) | m[ 9];
	state->timestamp = (state->timestamp << 8) | m[ 8];
	return 0;
}

#ifndef PIC30
int sbSetControlEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBSetControl* state, int requestAck)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,
			SB_MSGID_SET_CONTROL | (requestAck?SB_MSGID_REQACK:0), handle);
	m[ 0] = UC(state->roll);
	m[ 1] = UC(state->roll >> 8);
	m[ 2] = UC(state->pitch);
	m[ 3] = UC(state->pitch >> 8);
	m[ 4] = UC(state->yaw);
	m[ 5] = UC(state->yaw >> 8);
	m[ 6] = UC(state->altitude);
	m[ 7] = UC(state->altitude >> 8);
	msg->len = 8;
	return sbFinalizeMessage(msg);
}
#endif

int sbSetControlDecode(const SBSerialisedMessage *msg, SBSetControl* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbSetControlDecode,SB_MSGID_SET_CONTROL,8);
	state->roll = shortofch2(m[0],m[1]);
	state->pitch = shortofch2(m[2],m[3]);
	state->yaw = shortofch2(m[4],m[5]);
	state->altitude = shortofch2(m[6],m[7]);
	return 0;
}

#ifndef PIC30
int sbRawControlEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBRawControl* state, int requestAck)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,
			SB_MSGID_RAW_COMMAND | (requestAck?SB_MSGID_REQACK:0), handle);
	m[ 0] = UC(state->motor1);
	m[ 1] = UC(state->motor1 >> 8);
	m[ 2] = UC(state->motor2);
	m[ 3] = UC(state->motor2 >> 8);
	m[ 4] = UC(state->servo1);
	m[ 5] = UC(state->servo1 >> 8);
	m[ 6] = UC(state->servo2);
	m[ 7] = UC(state->servo2 >> 8);
	msg->len = 8;
	return sbFinalizeMessage(msg);
}
#endif

int sbRawControlDecode(const SBSerialisedMessage *msg, SBRawControl* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbRawControlDecode,SB_MSGID_RAW_COMMAND,8);
	state->motor1 = shortofch2(m[0],m[1]);
	state->motor2 = shortofch2(m[2],m[3]);
	state->servo1 = shortofch2(m[4],m[5]);
	state->servo2 = shortofch2(m[6],m[7]);
	return 0;
}


#ifndef PIC30
int sbCfgRawControlEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBConfigureRawControl* cfg, int requestAck)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,
			SB_MSGID_CFG_RAW_CMD | (requestAck?SB_MSGID_REQACK:0), handle);
	m[ 0] = cfg->speedprofile1;
	m[ 1] = cfg->speedprofile2;
	msg->len = 2;
	return sbFinalizeMessage(msg);
}
#endif

int sbCfgRawControlDecode(const SBSerialisedMessage *msg, SBConfigureRawControl* cfg)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbConfigureRawControlDecode,SB_MSGID_CFG_RAW_CMD,2);
	cfg->speedprofile1 = m[0];
	cfg->speedprofile2 = m[1];
	return 0;
}

#ifndef PIC30
int sbCfgTimeoutEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBConfigureTimeout* state, int requestAck)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,SB_MSGID_CFG_TIMEOUT | (requestAck?SB_MSGID_REQACK:0), handle);
	m[ 0] = UC(state->controlTimeout);
	m[ 1] = UC(state->controlTimeout >> 8);
	m[ 2] = UC(state->watchdogTimeout);
	m[ 3] = UC(state->watchdogTimeout >> 8);
	msg->len = 4;
	return sbFinalizeMessage(msg);
}
#endif

int sbCfgTimeoutDecode(const SBSerialisedMessage *msg, SBConfigureTimeout* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbSetControlDecode,SB_MSGID_CFG_TIMEOUT,4);
	state->controlTimeout = shortofch2(m[0],m[1]);
	state->watchdogTimeout = shortofch2(m[2],m[3]);
	return 0;
}


int sbTrimModeEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBTrimModeMessage* state, int setRequest)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg, SB_MSGID_TRIMMODE | (setRequest?SB_MSGID_REQACK:SB_MSGID_REPLY), handle);
	m[ 0] = state->trimMode;
	m[ 1] = UC(state->rollTrim);
	m[ 2] = UC(state->rollTrim >> 8);
	m[ 3] = UC(state->pitchTrim);
	m[ 4] = UC(state->pitchTrim >> 8);
	msg->len = 5;
	return sbFinalizeMessage(msg);
}

int sbTrimModeDecode(const SBSerialisedMessage *msg, SBTrimModeMessage* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbSetControlDecode,SB_MSGID_TRIMMODE,5);
	state->trimMode = m[0];
	state->rollTrim = shortofch2(m[1],m[2]);
	state->pitchTrim = shortofch2(m[3],m[4]);
	return 0;
}

const char* sbTrimModeString(unsigned char mode) {
	switch (mode) {
		case SB_TRIM_FROM_RC: return "From RC"; 
		case SB_TRIM_SOFTWARE: return "Software";
		default: return "Invalid";
	}
}

int sb6DofPoseEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SB6DofPoseMessage* pose, int reply)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	unsigned long t;
	sbInitializeMessage(msg, SB_MSGID_6DOF_POSE | (reply?SB_MSGID_REPLY:0), handle);
	t = pose->x;
	m[ 0] = UC(t); t >>= 8;
	m[ 1] = UC(t); t >>= 8;
	m[ 2] = UC(t); t >>= 8;
	m[ 3] = UC(t); 
	t = pose->y;
	m[ 4] = UC(t); t >>= 8;
	m[ 5] = UC(t); t >>= 8;
	m[ 6] = UC(t); t >>= 8;
	m[ 7] = UC(t); 
	t = pose->z;
	m[ 8] = UC(t); t >>= 8;
	m[ 9] = UC(t); t >>= 8;
	m[ 10] = UC(t); t >>= 8;
	m[ 11] = UC(t); 
	m[ 12] = UC(pose->roll);
	m[ 13] = UC(pose->roll >> 8);
	m[ 14] = UC(pose->pitch);
	m[ 15] = UC(pose->pitch >> 8);
	m[ 16] = UC(pose->yaw);
	m[ 17] = UC(pose->yaw >> 8);
	msg->len = 18;
	return sbFinalizeMessage(msg);
}

#define UL(x) ((unsigned long)(x))
#define ulongofch4(a,b,c,d) (UL(a) | (UL(b) << 8) | (UL(c) << 16) | (UL(d) << 24))
int sb6DofPoseDecode(const SBSerialisedMessage *msg, SB6DofPoseMessage* pose)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbSetControlDecode,SB_MSGID_6DOF_POSE,18);
	pose->x = ulongofch4(m[0],m[1],m[2],m[3]);
	pose->y = ulongofch4(m[4],m[5],m[6],m[7]);
	pose->z = ulongofch4(m[8],m[9],m[10],m[11]);
	pose->roll = shortofch2(m[12],m[13]);
	pose->pitch = shortofch2(m[14],m[15]);
	pose->yaw = shortofch2(m[16],m[17]);
	return 0;
}

int sbConfigureBluetoothEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBConfigureBluetoothMessage* cfg, int setRequest)
{
	unsigned int i;
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg, SB_MSGID_CFG_BLUETOOTH | (setRequest?SB_MSGID_REQACK:SB_MSGID_REPLY), handle);
	i=4; while (i) {
		m[i] = cfg->code[i];
		i--;
	}
	m += 4;
	i=16; while (i) {
		m[i] = cfg->name[i];
		i--;
	}
	msg->len = 20;
	return sbFinalizeMessage(msg);
}

int sbConfigureBluetoothDecode(const SBSerialisedMessage *msg, 
		SBConfigureBluetoothMessage* cfg)
{
	unsigned int i;
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbSetControlDecode,SB_MSGID_CFG_BLUETOOTH,20);
	i=4; while (i) {
		cfg->code[i] = m[i];
		i--;
	}
	m += 4;
	i=16; while (i) {
		cfg->name[i] = m[i];
		i--;
	}
	return 0;
}

int sbCtrlParametersEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBControlParametersMessage* state, int setRequest)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg, SB_MSGID_CTRLPARM | (setRequest?SB_MSGID_REQACK:SB_MSGID_REPLY), handle);
	m[ 0] = UC(state->baseThrust);
	m[ 1] = UC(state->baseThrust >> 8);
	m[ 2] = UC(state->yawOffset);
	m[ 3] = UC(state->yawOffset >> 8);
	m[ 4] = UC(state->altitudeKp);
	m[ 5] = UC(state->altitudeKp >> 8);
	m[ 6] = UC(state->altitudeKi);
	m[ 7] = UC(state->altitudeKi >> 8);
	m[ 8] = UC(state->altitudeKd);
	m[ 9] = UC(state->altitudeKd >> 8);
	m[10] = UC(state->yawKp);
	m[11] = UC(state->yawKp >> 8);
	m[12] = UC(state->yawKi);
	m[13] = UC(state->yawKi >> 8);
	m[14] = UC(state->yawKd);
	m[15] = UC(state->yawKd >> 8);
	msg->len = 16;
	return sbFinalizeMessage(msg);
}

int sbCtrlParametersDecode(const SBSerialisedMessage *msg, SBControlParametersMessage* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbSetControlDecode,SB_MSGID_CTRLPARM,16);
	state->baseThrust = shortofch2(m[0],m[1]);
	state->yawOffset = shortofch2(m[2],m[3]);
	state->altitudeKp = shortofch2(m[4],m[5]);
	state->altitudeKi = shortofch2(m[6],m[7]);
	state->altitudeKd = shortofch2(m[8],m[9]);
	state->yawKp = shortofch2(m[10],m[11]);
	state->yawKi = shortofch2(m[12],m[13]);
	state->yawKd = shortofch2(m[14],m[15]);
	return 0;
}

int sbBasicReplyEncode(SBSerialisedMessage *msg, unsigned char handle, 
		unsigned char msgid, const SBBasicReplyMessage* state)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,SB_MSGID_REPLY | msgid, handle);
	m[0] = state->status;
	msg->len = 1;
	return sbFinalizeMessage(msg);
}

#ifndef PIC30
int sbBasicReplyDecode(const SBSerialisedMessage *msg, SBBasicReplyMessage* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
#ifdef DEFENSIVE
	if (!(msg->msgid & SB_MSGID_REPLY)) {
		PRINTF("sbBasicReplyDecode: invalid msgid\n");
		return -1;
	}
	if (msg->len != 1) {
		PRINTF("sbBasicReplyDecode: invalid msg len\n");
		return -1;
	}
#endif
	state->status = m[0];
	return 0;
}
#endif

int sbStringMsgEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBStringMessage* text)
{
	unsigned int i;
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,SB_MSGID_STRING, handle);
	for (i=0;i<SB_STRING_MESSAGE_LENGTH;i++) {
		m[i] = text->text[i];
	}
	msg->len = SB_STRING_MESSAGE_LENGTH;
	return sbFinalizeMessage(msg);
}

int sbStringMsgDecode(const SBSerialisedMessage *msg, SBStringMessage* text)
{
	unsigned int i;
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbStringMsgDecode,SB_MSGID_CUSTOM,SB_STRING_MESSAGE_LENGTH);
	for (i=0;i<SB_STRING_MESSAGE_LENGTH;i++) {
		text->text[i] = m[i];
	}
	return 0;
}

int sbCustomMsgEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBCustomMessage* data, int reply)
{
	unsigned int i;
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,SB_MSGID_CUSTOM|(reply?SB_MSGID_REPLY:0), handle);
	for (i=0;i<SB_STRING_MESSAGE_LENGTH;i++) {
		m[i] = data->text[i];
	}
	msg->len = SB_STRING_MESSAGE_LENGTH;
	return sbFinalizeMessage(msg);
}

int sbCustomMsgDecode(const SBSerialisedMessage *msg, SBCustomMessage* text)
{
	unsigned int i;
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbCustomMsgDecode,SB_MSGID_CUSTOM,SB_STRING_MESSAGE_LENGTH);
	for (i=0;i<SB_STRING_MESSAGE_LENGTH;i++) {
		text->text[i] = m[i];
	}
	return 0;
}

int sbVersionMsgEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBVersionStatus* version)
{
	unsigned int i;
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,SB_MSGID_GET_VERSION | SB_MSGID_REPLY, handle);
	m[0] = UC(version->apiVersion);
	m[1] = UC(version->apiVersion >> 8);
	m[2] = UC(version->controllerVersion);
	m[3] = UC(version->controllerVersion >> 8);
	m += 4;
	for (i=0;i<SB_COMPILE_TIME_LENGTH;i++) {
		m[i] = version->compileTime[i];
	}
	m += SB_COMPILE_TIME_LENGTH;
	for (i=0;i<SB_IMU_VERSION_LENGTH;i++) {
		m[i] = version->imuVersion[i];
	}
	msg->len = SB_COMPILE_TIME_LENGTH + SB_IMU_VERSION_LENGTH + 4;
	return sbFinalizeMessage(msg);
}

#ifndef PIC30
int sbVersionMsgDecode(const SBSerialisedMessage *msg, SBVersionStatus* version)
{
	unsigned int i;
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbVersionMsgDecode,SB_MSGID_GET_VERSION,
			SB_COMPILE_TIME_LENGTH+SB_IMU_VERSION_LENGTH+4);
	version->apiVersion = shortofch2(m[0],m[1]);
	version->controllerVersion = shortofch2(m[2],m[3]);
	m += 4;
	for (i=0;i<SB_COMPILE_TIME_LENGTH;i++) {
		version->compileTime[i] = m[i];
	}
	m += SB_COMPILE_TIME_LENGTH;
	for (i=0;i<SB_IMU_VERSION_LENGTH;i++) {
		version->imuVersion[i] = m[i];
	}
	return 0;
}
#endif


int sbSensorListEncode(SBSerialisedMessage *msg, unsigned char handle, 
		const SBSensorListMessage* state)
{
	unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	sbInitializeMessage(msg,SB_MSGID_SENSORLIST | SB_MSGID_REPLY, handle);
	m[0] = UC(state->content[1]);
	m[1] = UC(state->content[1] >> 8);
	m[2] = UC(state->content[0]);
	m[3] = UC(state->content[0] >> 8);
	msg->len = 4;
	return sbFinalizeMessage(msg);
}

#ifndef PIC30
int sbSensorListDecode(const SBSerialisedMessage *msg, SBSensorListMessage* state)
{
	const unsigned char *m = msg->data+SB_MESSAGE_HEADER_LEN;
	CHECK_DEC(sbSensorListDecode,(SB_MSGID_SENSORLIST),4);
	state->content[1]= shortofch2(m[0],m[1]);
	state->content[0]= shortofch2(m[2],m[3]);
	return 0;
}
#endif

int sbValidateMessage(SBSerialisedMessage *sm)
{
	unsigned char crc = computeCRC(sm);
	if (sm->len > SB_MAX_MESSAGE_SIZE-SB_MESSAGE_HEADER_LEN) {
		return -3;
	}

	if (sm->len+SB_MESSAGE_HEADER_LEN >= SB_MAX_MESSAGE_SIZE) {
#ifndef PIC30
		PRINTF("sbValidateMessage: invalid size\n");
#endif
		return -2;
	}
	if (crc != sm->crc) {
#ifndef PIC30
		sbDumpMessage("Validation failed:",sm);
		PRINTF("sbValidateMessage(%02X): invalid CRC (got %02X expected %02X)\n",sm->msgid,sm->crc,crc);
#endif
		return -1;
	}
	return 0;
}

int sbDecodeMessage(SBSerialisedMessage *sm, void *dest)
{
	switch (sm->msgid) {
		case SB_MSGID_CONNECT:
			memcpy(dest,sm,sizeof(SBSerialisedMessage));
			return 0;
		case SB_MSGID_DISCONNECT:
			memcpy(dest,sm,sizeof(SBSerialisedMessage));
			return 0;
		case SB_MSGID_SENSORLIST:
			memcpy(dest,sm,sizeof(SBSerialisedMessage));
			return 0;
		case SB_MSGID_CFG_OAVOID:
			return sbCfgOADecode(sm,(SBConfigureObstAvoid*)dest);
		case SB_MSGID_CFG_COMM:
			return sbCfgCommDecode(sm,(SBConfigureCommunication*)dest);
		case SB_MSGID_CFG_CONTROL:
			return sbCfgControlDecode(sm,(SBConfigureControl*)dest);
		case SB_MSGID_STATE:
			return sbReqStateDecode(sm,(SBRequestState*)dest);
		case SB_MSGID_SET_NAVMODE:
			return sbSetNavDecode(sm,(SBSetNavigationMode*)dest);
		case SB_MSGID_SET_CONTROL:
			return sbSetControlDecode(sm,(SBSetControl*)dest);

#ifndef PIC30
		// The PIC will never have to decode answers
		case SB_MSGID_REPLY | SB_MSGID_CONNECT:
			return sbBasicReplyDecode(sm,(SBBasicReplyMessage*)dest);
		case SB_MSGID_REPLY | SB_MSGID_DISCONNECT:
			return sbBasicReplyDecode(sm,(SBBasicReplyMessage*)dest);
		case SB_MSGID_REPLY | SB_MSGID_SENSORLIST:
			return sbSensorListDecode(sm,(SBSensorListMessage*)dest);
		case SB_MSGID_REPLY | SB_MSGID_CFG_OAVOID:
			return sbBasicReplyDecode(sm,(SBBasicReplyMessage*)dest);
		case SB_MSGID_REPLY | SB_MSGID_CFG_COMM:
			return sbBasicReplyDecode(sm,(SBBasicReplyMessage*)dest);
		case SB_MSGID_REPLY | SB_MSGID_CFG_CONTROL:
			return sbBasicReplyDecode(sm,(SBBasicReplyMessage*)dest);
		case SB_MSGID_REPLY | SB_MSGID_STATE:
			return sbStateDecode(sm,(SBHeliStateRaw*)dest);
		case SB_MSGID_REPLY | SB_MSGID_SET_NAVMODE:
			return sbBasicReplyDecode(sm,(SBBasicReplyMessage*)dest);
		case SB_MSGID_REPLY | SB_MSGID_SET_CONTROL:
			return sbBasicReplyDecode(sm,(SBBasicReplyMessage*)dest);
#endif
		default:
			return -1;
	}
}


#ifdef SBC_HAS_IO

// for sbContentPrint
#include "com/sbapi.h" 

int sbSaveMessage(SBSerialisedMessage *sm, FILE * fp)
{
	if (fwrite(sm->data,sm->len+4,1,fp) == (sm->len+4)) {
		return -1;
	}
	return 0;
}

int sbLoadMessage(SBSerialisedMessage *sm, FILE * fp)
{
	int crc,n;
	memset(sm,0,sizeof(SBSerialisedMessage));
	if (fread(sm->data,1,3,fp)!=3) {
		PRINTF("loadMessage: failed to read message header\n");
		return -1;
	}
	sm->msgid = sm->data[0];
	sm->handle = sm->data[1];
	sm->len = sm->data[2];
	n = sm->len + 1;
	if (fread(sm->data+3,1,n,fp) != n) {
		PRINTF("loadMessage: failed to read message body\n");
		return -2;
	}
	sm->crc = sm->data[sm->len+3];
	crc = computeCRC(sm);
	if (crc != sm->crc) {
		PRINTF("loadMessage: invalid CRC\n");
		return -3;
	}
	return 0;
}


void sbPrintMessageHead(FILE *fp, SBSerialisedMessage *sm)
{
	fprintf(fp,"Message ");
	switch (sm->msgid & 0x7f) {
		case SB_MSGID_CONNECT    :
			fprintf(fp,"SB_MSGID_CONNECT    ");
			break;
		case SB_MSGID_DISCONNECT :
			fprintf(fp,"SB_MSGID_DISCONNECT ");
			break;
		case SB_MSGID_SENSORLIST :
			fprintf(fp,"SB_MSGID_SENSORLIST ");
			break;
		case SB_MSGID_CFG_OAVOID :
			fprintf(fp,"SB_MSGID_CFG_OAVOID ");
			break;
		case SB_MSGID_CFG_COMM   :
			fprintf(fp,"SB_MSGID_CFG_COMM   ");
			break;
		case SB_MSGID_CFG_CONTROL:
			fprintf(fp,"SB_MSGID_CFG_CONTROL");
			break;
		case SB_MSGID_STATE      :
			fprintf(fp,"SB_MSGID_STATE      ");
			break;
		case SB_MSGID_SET_NAVMODE:
			fprintf(fp,"SB_MSGID_SET_NAVMODE");
			break;
		case SB_MSGID_SET_CONTROL:
			fprintf(fp,"SB_MSGID_SET_CONTROL");
			break;
		default:
			fprintf(fp,"SB_MSGID_???????????");
			break;
	}
	fprintf(fp,"(%s) ",(sm->msgid & SB_MSGID_REPLY)?"reply":"request");
	fprintf(fp,": handle %d len %d crc %02X\n",sm->handle,sm->len,sm->crc);
}

void sbCfgOAPrint(FILE *fp, SBConfigureObstAvoid* state)
{
	fprintf(fp,"Obst. Avoid: Vertical %d Horizontal %d\n",
			(state->oavoidMode & SB_OA_VERTICAL)?1:0,
			(state->oavoidMode & SB_OA_HORIZONTAL)?1:0);
}

void sbCfgCommPrint(FILE *fp, SBConfigureCommunication* state)
{
	fprintf(fp,"Continuous communication: %d\n",state->commMode);
	if (state->commMode) {
		unsigned long content = ((unsigned long)state->content[0]) |
			(((unsigned long)state->content[1]) << 16);
		fprintf(fp,"\tFrequency %d Content %04X%04X\n\t",state->frequency,state->content[0],state->content[1]);
		sbContentPrint(fp,content);
		fprintf(fp,"\n");
	}
}

void sbCfgControlPrint(FILE *fp, SBConfigureControl* state)
{
	fprintf(fp,"Control Modes | Roll %s | Pitch %s | Yaw %s | Alt. %s |\n",
			sbCtrlModeString(state->roll), sbCtrlModeString(state->pitch), 
			sbCtrlModeString(state->yaw), sbCtrlModeString(state->altitude));
}

void sbCfgTimeoutPrint(FILE *fp, SBConfigureTimeout* state)
{
	fprintf(fp,"Timeouts: Control %d Watchdog %d\n",
			state->controlTimeout, state->watchdogTimeout);
}

void sbReqStatePrint(FILE *fp, SBRequestState* state)
{
	unsigned long content = ((unsigned long)state->content[0]) |
		(((unsigned long)state->content[1]) << 16);
	fprintf(fp,"Requested state: Content %04X%04X:",state->content[0],state->content[1]);
	sbContentPrint(fp,content);
	fprintf(fp,"\n");
}

void sbSetNavPrint(FILE *fp, SBSetNavigationMode* state)
{
	fprintf(fp,"Navigation mode: %d\n",state->mode);
}

void sbSetControlPrint(FILE *fp, SBSetControl* state)
{
	fprintf(fp,"Control: Roll %d Pitch %d Yaw %d Alt. %d\n",
			state->roll, state->pitch, state->yaw,
			state->altitude);
}

void sbRawControlPrint(FILE *fp, SBRawControl* state)
{
	fprintf(fp,"Raw Control: Motor1 %d Motor2 %d Servo1 %d Servo2 %d\n",
			state->motor1, state->motor2, state->servo1, state->servo2);
}

void sbTrimModePrint(FILE *fp, SBTrimModeMessage* state)
{
	fprintf(fp,"Trim Mode: %s Roll %d Pitch %d\n",
			sbTrimModeString(state->trimMode),state->rollTrim,state->pitchTrim);
}

void sbCtrlParametersPrint(FILE *fp, SBControlParametersMessage* state) 
{
	fprintf(fp,"Ctrl Parameters: Altitude Kh %d P %d I %d D %d Yaw Off %d P %d I %d D %d\n",
			state->baseThrust, state->altitudeKp, state->altitudeKi, state->altitudeKd,
			state->yawOffset, state->yawKp, state->yawKi, state->yawKd);
}

void sbBasicReplyPrint(FILE *fp, SBBasicReplyMessage* state)
{
	switch (state->status) {
		case SB_REPLY_OK: 
			fprintf(fp,"Reply: OK\n");
			break;
		case SB_REPLY_ERROR: 
			fprintf(fp,"Reply: ERROR\n");
			break;
		default:
			fprintf(fp,"Reply: Unknown\n");
			break;
	}
}

void sbSensorListPrint(FILE * fp, SBSensorListMessage *state) 
{
	fprintf(fp,"Sensor list: %04X%04X\n",state->content[0],state->content[1]);
}


#ifndef PIC30
void sbDumpMessage(const char * prefix, const SBSerialisedMessage *sm)
{
	unsigned int i;
	printf("%s Message: id %02X handle %d len %d [",prefix,sm->msgid,sm->handle,sm->len);
	for (i=0;i<sm->len;i++) {
		printf("%02X ",sm->data[i+SB_MESSAGE_HEADER_LEN]);
	}
	printf("] CRC %02X\n",sm->crc);
}
#endif

#endif // SBC_HAS_IO

#ifdef SBC_HAS_COMM

#ifdef PIC30
void sbDebugMessage(SBChannel *channel, 
		const char * prefix, SBSerialisedMessage *sm)
{
	unsigned int i;
	char buffer[128];
	sbChannelSendString(channel,prefix,100);
	sprintf(buffer,"Message: id %02X/%02X handle %d/%d len %d/%d [",
			sm->msgid,sm->data[0],
			sm->handle,sm->data[1],
			sm->len,sm->data[2]);
	sbChannelSendString(channel,buffer,100);
	for (i=0;i<sm->len;i++) {
		sprintf(buffer,"%02X ",sm->data[i+SB_MESSAGE_HEADER_LEN]);
		sbChannelSendString(channel,buffer,100);
	}
	sprintf(buffer,"] CRC %02X\n",sm->crc);
	sbChannelSendString(channel,buffer,100);
}

int sbWaitRawMessage(SBChannel *channel, int msgid, SBSerialisedMessage *sm, 
		unsigned int timeout_ms)
{
	unsigned char n = 0;
#ifdef DEBUG_LED
	e_led_clear();
	e_set_led(0,1);
#endif
    while (1) { 
        if (sbChannelWaitBuffer(channel, sm->data, 1, timeout_ms)) {
            return -1;
        }
        if (sm->data[0] == SB_MESSAGE_START_FRAME) {
            break;
        }
    }
	if (sbChannelWaitBuffer(channel, sm->data+1, SB_MESSAGE_HEADER_LEN-1, timeout_ms)) {
		return -1;
	} else {
		// printf(".");fflush(stdout);
		sm->msgid = sm->data[1];
		sm->handle = sm->data[2];
		sm->len = sm->data[3];
		n = sm->len + 1;
	}

	if (sm->len > SB_MAX_MESSAGE_SIZE-SB_MESSAGE_HEADER_LEN) {
		return -5;
	}
#ifdef DEBUG_LED
	e_set_led(1,1);
#endif
	if (sbChannelWaitBuffer(channel, sm->data+SB_MESSAGE_HEADER_LEN, n, timeout_ms)) {
		return -2;
	} else {
		// printf("!");fflush(stdout);
		sm->crc = sm->data[sm->len+SB_MESSAGE_HEADER_LEN];
	}
#ifdef DEBUG_LED
	e_set_led(2,1);
#endif
	if (sbValidateMessage(sm)) { 
		return -3; 
	}
	if ((msgid>=0) && (sm->msgid != msgid)) {
		return -4;
	}
#ifdef DEBUG_LED
	e_set_led(3,1);
#endif
	return 0;
}
#else
int sbWaitRawMessage(SBChannel *channel, int msgid, SBSerialisedMessage *sm, 
		unsigned int timeout_ms)
{
	unsigned char n = 0;
	unsigned int numtries = 10;
	while (numtries) {
		numtries --;
        while (1) { 
            if (sbChannelWaitBuffer(channel, sm->data, 1, timeout_ms)) {
                VPRINTF("Ouch\n");
                return -1;
            }
            if (sm->data[0] == SB_MESSAGE_START_FRAME) {
                break;
            }
            VPRINTF("Discarding %02X\n",sm->data[0]);
        }
        if (sbChannelWaitBuffer(channel, sm->data+1, SB_MESSAGE_HEADER_LEN-1, timeout_ms)) {
			VPRINTF("Arghhh\n");
			return -1;
		} else {
			// printf(".");fflush(stdout);
			sm->msgid = sm->data[1];
			sm->handle = sm->data[2];
			sm->len = sm->data[3];
			n = sm->len + 1;
            // printf("Packet %02X H%d len %d\n",sm->msgid,sm->handle,sm->len);
		}
		if (sm->len > SB_MAX_MESSAGE_SIZE-SB_MESSAGE_HEADER_LEN) {
			VPRINTF("Rotten packet\n");
			return -5;
		}
		if (sbChannelWaitBuffer(channel, sm->data+SB_MESSAGE_HEADER_LEN, n, timeout_ms)) {
			VPRINTF("Crack\n");
			return -2;
		} else {
			// printf("!");fflush(stdout);
			sm->crc = sm->data[sm->len+SB_MESSAGE_HEADER_LEN];
		}
		if (sbValidateMessage(sm)) { 
			VPRINTF("Pop\n");
			return -3; 
		}
		if ((msgid>=0) && (sm->msgid != msgid)) {
			VPRINTF("Message with incorrect id (%d instead of %d), continuing\n",
					sm->msgid, msgid);
			continue;
		}
#ifdef DEBUG_MESSAGE
		sbDumpMessage("received:",sm);
		printf("Message received with correct id\n");
#endif
		return 0;
	}
	return -1;
}
#endif


int sbWaitMessage(SBChannel *channel, unsigned char msgid, void *dest, 
		unsigned int timeout_ms)
{
	SBSerialisedMessage sm;
	if (sbWaitRawMessage(channel,msgid,&sm,timeout_ms)) {
		PRINTF("Failed to receive raw message\n");
		return -1;
	}
	return sbDecodeMessage(&sm,dest);
}


int sbSendMessage(SBChannel *channel, SBSerialisedMessage *sm, 
		unsigned int timeout_ms)
{
#if PIC30
#ifdef DEBUG_LED
	e_led_clear();
	e_set_led(1,1); e_set_led(2,1);
#endif
#else
#ifdef DEBUG_MESSAGE
	sbDumpMessage("sending: ",sm);
#endif
#endif
	return sbChannelSendAll(channel,sm->data,sm->len+SB_MESSAGE_HEADER_LEN+1,timeout_ms);
}



#endif // SBC_HAS_COMM
