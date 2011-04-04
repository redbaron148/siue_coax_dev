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

#ifdef PIC30
typedef unsigned int size_t;
#define NULL ((void*)0)
#else
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#endif

#include <stdarg.h>

#include "com/sbmessage.h"

#ifdef WIN32
#ifdef DEFENSIVE
#define PRINTF(X,...) printf(X,__VA_ARGS__)
#else
#define PRINTF(X,...) 
#endif
#else
#ifdef DEFENSIVE
#define PRINTF(X...) printf(X)
#else
#define PRINTF(X...) 
#endif
#endif

typedef struct {
	unsigned short id;
	unsigned short bank;
	unsigned short mask[2];    // needed to represent a 32 bit mask
	unsigned short maskorder[2]; // needed to represent a 32 bit mask
	unsigned short datalen;
} FLAG_DATA;

static 
FLAG_DATA fdata[] = {
	{ /* SBC_MODES			*/  0,	0,	{0x0000u,0x0001u},	{0x0000,0x0000}, 2},
	{ /* SBC_TIMESTAMP		*/  1,	0,	{0x0000u,0x0002u},	{0x0000,0x0001}, 2},		
	{ /* SBC_RPY			*/  2,	0,	{0x0000u,0x0004u},	{0x0000,0x0003}, 3},
	{ /* SBC_GYRO			*/  3,	0,	{0x0000u,0x0008u},	{0x0000,0x0007}, 3},
	{ /* SBC_ACCEL			*/  4,	0,	{0x0000u,0x0010u},	{0x0000,0x000f}, 3},
	{ /* SBC_MAGNETO		*/  5,	0,	{0x0000u,0x0020u},	{0x0000,0x001f}, 3},
	{ /* SBC_IMUTEMP		*/  6,	0,	{0x0000u,0x0040u},	{0x0000,0x003f}, 1},
	{ /* SBC_ALTITUDE		*/  7,	0,	{0x0000u,0x0080u},	{0x0000,0x007f}, 2},
	{ /* SBC_PRESSURE		*/  8,	0,	{0x0000u,0x0100u},	{0x0000,0x00ff}, 1},
	{ /* SBC_HRANGES		*/  9,	0,	{0x0000u,0x0200u},	{0x0000,0x01ff}, 4},
	{ /* SBC_XY_REL			*/ 10,	0,	{0x0000u,0x0400u},	{0x0000,0x03ff}, 2},
	{ /* SBC_BATTERY		*/ 11,	0,	{0x0000u,0x0800u},	{0x0000,0x07ff}, 1},
	{ /* SBC_TIMEOUT		*/ 12,	0,	{0x0000u,0x1000u},	{0x0000,0x0fff}, 2},
	{ /* SBC_O_ATTITUDE		*/ 13,	0,	{0x0000u,0x2000u},	{0x0000,0x1fff}, 3},
	{ /* SBC_O_ALTITUDE		*/ 14,	0,	{0x0000u,0x4000u},	{0x0000,0x3fff}, 1},
	{ /* SBC_O_TOL			*/ 15,	0,	{0x0000u,0x8000u},	{0x0000,0x7fff}, 1},
	{ /* SBC_O_XY			*/ 16,	1,	{0x0001u,0x0000u},	{0x0000,0xffff}, 2},
	{ /* SBC_O_OBSAVOID		*/ 17,	1,	{0x0002u,0x0000u},	{0x0001,0xffff}, 2},
	{ /* SBC_CHANNELS		*/ 18,	1,	{0x0004u,0x0000u},	{0x0003,0xffff}, 8},
	{ /* SBC_COAXSPEED		*/ 19,	1,	{0x0008u,0x0000u},	{0x0007,0xffff}, 3},
	{ /* RESERVED			*/ 20,	1,	{0x0010u,0x0000u},	{0x000f,0xffff}, 0},
	{ /* RESERVED			*/ 21,	1,	{0x0020u,0x0000u},	{0x001f,0xffff}, 0},
	{ /* RESERVED			*/ 22,	1,	{0x0040u,0x0000u},	{0x003f,0xffff}, 0},
	{ /* RESERVED			*/ 23,	1,	{0x0080u,0x0000u},	{0x007f,0xffff}, 0}, // Last of the 24 bits of content
	{ /* UNUSED  			*/ 24,	1,	{0x0000u,0x0000u},	{0xffff,0xffff}, 0}, // Other mask are for multibit operations
	{ /* UNUSED  			*/ 25,	1,	{0x0000u,0x0000u},	{0xffff,0xffff}, 0},
	{ /* UNUSED  			*/ 26,	1,	{0x0000u,0x0000u},	{0xffff,0xffff}, 0},
	{ /* SBC_ALL  			*/ 27,	0,	{0x0007u,0xffffu},	{0xffff,0xffff}, 0},
	{ /* SBC_IMU_ALL		*/ 28,	0,	{0x0000u,0x007Cu},	{0xffff,0xffff}, 0},
	{ /* SBC_RANGES_ALL		*/ 29,	0,	{0x0000u,0x0280u},	{0xffff,0xffff}, 0},
	{ /* SBC_ALTITUDE_ALL	*/ 30,	0,	{0x0000u,0x0180u},	{0xffff,0xffff}, 0},
	{ /* SBC_OUTPUT_ALL		*/ 31,	0,	{0x0003u,0xE000u},	{0xffff,0xffff}, 0},
};
#define SBC_LAST_ITEM SBC_COAXSPEED
#define SBC_LAST_ITEM_NAME "SBC_COAXSPEED"

typedef struct {
	unsigned char len;
	unsigned short content[2];
	unsigned char *msg;
} StatusMessage;


#ifdef DEFENSIVE
static
int checkCoherence(unsigned short content[2], int whatid)
{
	// PRINTF("checkCoherence: %04X%04X %d\n",content[0],content[1],whatid);
	// sbContent16bPrint(stdout,content);
	if (whatid < 0) {
		// whatid must have been returned by getFlagId
		PRINTF("checkCoherence: what (%d) < 0\n",whatid);
		return -1;
	}
	if ((content[0] & ~fdata[whatid].maskorder[0]) || 
			(content[1] & ~fdata[whatid].maskorder[1])) {
		// for consistency, no bit above the bit in "what" should 
		// be set in content
		PRINTF("checkCoherence: %04X%04X inconsistent with %04X%04X\n",
				content[0],content[1], fdata[whatid].maskorder[0],fdata[whatid].maskorder[1]);
		return -1;
	}
	return 0;
}
#endif


#define TESTCONTENT(content,what) \
	(content[0] & fdata[what].mask[0]) || (content[1] & fdata[what].mask[1])


int sbTestContent(unsigned short content[2], unsigned int what)
{
#ifdef DEFENSIVE
	if (what < 0) {
		PRINTF("sbTestContent: what (%d) < 0\n",what);
		return -1;
	}
	if (what > 31) {
		PRINTF("sbTestContent: what (%d) > 31\n",what);
		return -1;
	}
#endif
	return TESTCONTENT(content,what);
}

int sbAddOneContent(unsigned short content[2], int what)
{
	content[0] |= fdata[what].mask[0];
	content[1] |= fdata[what].mask[1];
	return 0;
}

static
int sbAddShortArray(StatusMessage *sm, unsigned int what,
		const signed short *buffer, unsigned int len)
{
	unsigned int lit = len;
	unsigned char *dest = sm->msg + sm->len;
#ifdef DEFENSIVE
	if (what < 0) {
		PRINTF("sbAddShortArray: what (%d) < 0\n",what);
		return -1;
	}
	if (what > SBC_LAST_ITEM) {
		PRINTF("sbAddShortArray: what (%d) > %s\n",what,SBC_LAST_ITEM_NAME);
		return -1;
	}
	if (checkCoherence(sm->content,what)) {
		PRINTF("sbAddShortArray: invalid coherence\n");
		return -1;
	}
#endif
	while (lit>0) {
		*dest = (unsigned char)*buffer;
		dest++; 
		*dest = (unsigned char)(*buffer>>8);
		dest++; 
		buffer++;
		lit --;
	}
	dest = sm->msg + sm->len;
	sm->len += 2*len;
	return sbAddOneContent(sm->content,what);	
}

static
int sbAddOneValue(StatusMessage *sm, unsigned int what,unsigned short value)
{
#ifdef DEFENSIVE
	if (what < 0) {
		PRINTF("sbAddValues: what (%d) < 0\n",what);
		return -1;
	}
	if (what > SBC_LAST_ITEM) {
		PRINTF("sbAddValues: what (%d) > %s\n",what,SBC_LAST_ITEM_NAME);
		return -1;
	}
	if (checkCoherence(sm->content,what)) {
		PRINTF("sbAddValues: invalid coherence\n");
		return -1;
	}
#endif
	sm->msg[sm->len] = (unsigned char)value;
	sm->msg[sm->len+1] = (unsigned char)(value>>8);
	sm->len += 2;
	return sbAddOneContent(sm->content,what);	
}

static
int sbAddTwoValues(StatusMessage *sm, unsigned int what,unsigned short v1,unsigned short v2)
{
#ifdef DEFENSIVE
	if (what < 0) {
		PRINTF("sbAddValues: what (%d) < 0\n",what);
		return -1;
	}
	if (what > SBC_LAST_ITEM) {
		PRINTF("sbAddValues: what (%d) > %s\n",what,SBC_LAST_ITEM_NAME);
		return -1;
	}
	if (checkCoherence(sm->content,what)) {
		PRINTF("sbAddValues: invalid coherence\n");
		return -1;
	}
#endif
	sm->msg[sm->len+0] = (unsigned char)v1;
	sm->msg[sm->len+1] = (unsigned char)(v1>>8);
	sm->msg[sm->len+2] = (unsigned char)v2;
	sm->msg[sm->len+3] = (unsigned char)(v2>>8);
	sm->len += 4;
	return sbAddOneContent(sm->content,what);	
}

static
int sbAddThreeValues(StatusMessage *sm, unsigned int what,
		unsigned short v1, unsigned short v2, unsigned short v3)
{
#ifdef DEFENSIVE
	if (what < 0) {
		PRINTF("sbAddValues: what (%d) < 0\n",what);
		return -1;
	}
	if (what > SBC_LAST_ITEM) {
		PRINTF("sbAddValues: what (%d) > %s\n",what,SBC_LAST_ITEM_NAME);
		return -1;
	}
	if (checkCoherence(sm->content,what)) {
		PRINTF("sbAddValues: invalid coherence\n");
		return -1;
	}
#endif
	sm->msg[sm->len+0] = (unsigned char)v1;
	sm->msg[sm->len+1] = (unsigned char)(v1>>8);
	sm->msg[sm->len+2] = (unsigned char)v2;
	sm->msg[sm->len+3] = (unsigned char)(v2>>8);
	sm->msg[sm->len+4] = (unsigned char)v3;
	sm->msg[sm->len+5] = (unsigned char)(v3>>8);
	sm->len += 6;
	return sbAddOneContent(sm->content,what);	
}

static
int sbAddFourValues(StatusMessage *sm, unsigned int what,
		unsigned short v1, unsigned short v2, 
		unsigned short v3, unsigned short v4)
{
#ifdef DEFENSIVE
	if (what < 0) {
		PRINTF("sbAddValues: what (%d) < 0\n",what);
		return -1;
	}
	if (what > SBC_LAST_ITEM) {
		PRINTF("sbAddValues: what (%d) > %s\n",what,SBC_LAST_ITEM_NAME);
		return -1;
	}
	if (checkCoherence(sm->content,what)) {
		PRINTF("sbAddValues: invalid coherence\n");
		return -1;
	}
#endif
	sm->msg[sm->len+0] = (unsigned char)v1;
	sm->msg[sm->len+1] = (unsigned char)(v1>>8);
	sm->msg[sm->len+2] = (unsigned char)v2;
	sm->msg[sm->len+3] = (unsigned char)(v2>>8);
	sm->msg[sm->len+4] = (unsigned char)v3;
	sm->msg[sm->len+5] = (unsigned char)(v3>>8);
	sm->msg[sm->len+6] = (unsigned char)v4;
	sm->msg[sm->len+7] = (unsigned char)(v4>>8);
	sm->len += 8;
	return sbAddOneContent(sm->content,what);	
}

#ifndef PIC30
int sbAddContent(unsigned short content[2], int what, ...)
{
	va_list ap;
	va_start(ap,what);
	while (what>=0) {
#ifdef DEFENSIVE
		if (what < 0) {
			PRINTF("sbAddContent: what (%d) < 0\n",what);
			return -1;
		}
		if (what > 31) {
			PRINTF("sbAddContent: what (%d) > 31\n",what);
			return -1;
		}
#endif
		content[0] |= fdata[what].mask[0];
		content[1] |= fdata[what].mask[1];
		what = va_arg(ap,int);
	}
	va_end(ap);
	return 0;
}

#endif

#define SBC_MODE_NAV 0x0F
#define SBC_MODE_COMM 0x01
#define SBC_MODE_OAVOID 0x03
#define SBC_MODE_ROLL 0x07
#define SBC_MODE_PITCH 0x07
#define SBC_MODE_YAW 0x07
#define SBC_MODE_ALT 0x07

#define SBC_SHIFT_NAV 0
#define SBC_SHIFT_COMM 4
#define SBC_SHIFT_OAVOID 5

#define SBC_SHIFT_ROLL 0
#define SBC_SHIFT_PITCH 3
#define SBC_SHIFT_YAW 6
#define SBC_SHIFT_ALT 9 

#define sbAddModes(sm,nav,comm,oavoid,roll,pitch,yaw,alt) \
	sbAddTwoValues(sm,SBC_MODES,\
			(((unsigned short)(nav & SBC_MODE_NAV)) << SBC_SHIFT_NAV) | \
			(((unsigned short)(comm & SBC_MODE_COMM)) << SBC_SHIFT_COMM) | \
			(((unsigned short)(oavoid & SBC_MODE_OAVOID)) << SBC_SHIFT_OAVOID), \
			(((unsigned short)(roll & SBC_MODE_ROLL)) << SBC_SHIFT_ROLL) | \
			(((unsigned short)(pitch & SBC_MODE_PITCH)) << SBC_SHIFT_PITCH) | \
			(((unsigned short)(yaw & SBC_MODE_YAW)) << SBC_SHIFT_YAW) | \
			(((unsigned short)(alt & SBC_MODE_ALT)) << SBC_SHIFT_ALT) ) 

#define sbAddTimeout(sm, ctrlTO,wdTO) sbAddTwoValues(sm,SBC_TIMEOUT,ctrlTO,wdTO)
#define sbAddBattery(sm, batt) sbAddOneValue(sm,SBC_BATTERY,batt)
#define sbAddTimeStamp(sm, timestamp) sbAddTwoValues(sm,SBC_TIMESTAMP,\
		(unsigned short)timestamp, (unsigned short)(timestamp>>16))
#define sbAddRPY(sm,roll,pitch,yaw) sbAddThreeValues(sm,SBC_RPY,roll,pitch,yaw)
#define sbAddGyro(sm,roll,pitch,yaw) sbAddThreeValues(sm,SBC_GYRO,roll,pitch,yaw)
#define sbAddAccel(sm,x,y,z) sbAddThreeValues(sm,SBC_ACCEL,x,y,z)
#define sbAddMagneto(sm,x,y,z) sbAddThreeValues(sm,SBC_MAGNETO,x,y,z)
#define sbAddIMUTemp(sm,temp) sbAddOneValue(sm,SBC_IMUTEMP,temp)
#define sbAddAltitude(sm,z,zfilt) sbAddTwoValues(sm,SBC_ALTITUDE,z,zfilt)
#define sbAddPressure(sm,z) sbAddOneValue(sm,SBC_PRESSURE,z)
#define sbAddHRanges(sm,s1,s2,s3,s4) sbAddFourValues(sm,SBC_HRANGES,s1,s2,s3,s4)
#define sbAddXYRel(sm,x,y) sbAddTwoValues(sm,SBC_XY_REL,x,y)

#define sbAddOutAttitude(sm,roll,pitch,yaw) \
	sbAddThreeValues(sm,SBC_O_ATTITUDE,roll,pitch,yaw)

#define sbAddOutAltCtrl(sm,uz) \
	sbAddOneValue(sm,SBC_O_ALTITUDE,uz)

#define sbAddOutAltTOL(sm,uz) \
	sbAddOneValue(sm,SBC_O_TOL,uz)

#define sbAddOutXY(sm,x,y) sbAddTwoValues(sm,SBC_O_XY,x,y)

#define sbAddOutOA(sm,x,y) sbAddTwoValues(sm,SBC_O_OAVOID,x,y)

#define sbAddChannels(sm,channels) sbAddShortArray(sm,SBC_CHANNELS,channels,8)

#define sbAddCoaxSpeed(sm,ce) sbAddThreeValues(sm,SBC_COAXSPEED,\
        ce.state | (((unsigned short)ce.light) << 8), \
        ce.vel_x,ce.vel_y)

#define UC(X) ((unsigned char)((X)))


#define DO(c) if(c) {counter+=1;PRINTF("Command "#c" failed\n");return -counter;}

int sbStateEncode(SBSerialisedMessage *smsg, unsigned char handle, const SBHeliStateRaw* state)
{
	int counter=0;
	StatusMessage sm = {4, {0,0}, NULL};
	sbInitializeMessage(smsg,SB_MSGID_STATE | SB_MSGID_REPLY, handle);
	sm.msg = smsg->data+SB_MESSAGE_HEADER_LEN;
	sm.msg[0] = UC(state->errorFlags);
	sm.msg[1] = UC(state->content[1]);
	sm.msg[2] = UC((state->content[1] >> 8));
	sm.msg[3] = UC(state->content[0]);
	if (TESTCONTENT(state->content,SBC_MODES)) {
		DO(sbAddModes(&sm,state->mode.navigation,state->mode.communication,
				state->mode.oavoid, state->mode.rollAxis, state->mode.pitchAxis,
				state->mode.yawAxis, state->mode.altAxis));
	}
	if (TESTCONTENT(state->content,SBC_TIMESTAMP)) {
		DO(sbAddTimeStamp(&sm,state->timeStamp));
	}
	if (TESTCONTENT(state->content,SBC_RPY)) {
		DO(sbAddRPY(&sm,state->roll,state->pitch,state->yaw));
	}
	if (TESTCONTENT(state->content,SBC_GYRO)) {
		DO(sbAddGyro(&sm,state->gyro[0],state->gyro[1],state->gyro[2]));
	}
	if (TESTCONTENT(state->content,SBC_ACCEL)) {
		DO(sbAddAccel(&sm,state->accel[0],state->accel[1],state->accel[2]));
	}
	if (TESTCONTENT(state->content,SBC_MAGNETO)) {
		DO(sbAddMagneto(&sm,state->magneto[0],state->magneto[1],state->magneto[2]));
	}
	if (TESTCONTENT(state->content,SBC_IMUTEMP)) {
		DO(sbAddIMUTemp(&sm,state->imutemp));
	}
	if (TESTCONTENT(state->content,SBC_ALTITUDE)) {
		DO(sbAddAltitude(&sm,state->zrange,state->zfiltered));
	}
	if (TESTCONTENT(state->content,SBC_PRESSURE)) {
		DO(sbAddPressure(&sm,state->pressure));
	}
	if (TESTCONTENT(state->content,SBC_HRANGES)) {
		DO(sbAddHRanges(&sm,state->hranges[0],state->hranges[1],
					state->hranges[2],state->hranges[3]));
	}
	if (TESTCONTENT(state->content,SBC_XY_REL)) {
		DO(sbAddXYRel(&sm,state->xrel,state->yrel));
	}
	if (TESTCONTENT(state->content,SBC_BATTERY)) {
		DO(sbAddBattery(&sm,state->battery));
	}
	if (TESTCONTENT(state->content,SBC_TIMEOUT)) {
		DO(sbAddTimeout(&sm,state->controlTimeout,state->watchdogTimeout));
	}

	if (TESTCONTENT(state->content,SBC_O_ATTITUDE)) {
		DO(sbAddOutAttitude(&sm,state->o_attitude[0],state->o_attitude[1],state->o_attitude[2]));
	}
	if (TESTCONTENT(state->content,SBC_O_ALTITUDE)) {
		DO(sbAddOutAltCtrl(&sm,state->o_altitude));
	}
	if (TESTCONTENT(state->content,SBC_O_TOL)) {
		DO(sbAddOutAltTOL(&sm,state->o_tol));
	}
	if (TESTCONTENT(state->content,SBC_O_XY)) {
		DO(sbAddOutXY(&sm,state->o_xy[0],state->o_xy[1]));
	}
	if (TESTCONTENT(state->content,SBC_O_OAVOID)) {
		DO(sbAddOutOA(&sm,state->o_oavoid[0],state->o_oavoid[1]));
	}
	if (TESTCONTENT(state->content,SBC_CHANNELS)) {
		DO(sbAddChannels(&sm,state->rcChannel));
	}
	if (TESTCONTENT(state->content,SBC_COAXSPEED)) {
		DO(sbAddCoaxSpeed(&sm,state->coaxspeed));
	}

#ifndef PIC30
    assert(sm.len < SB_MAX_MESSAGE_SIZE);
    // printf("Message len: %d\n",sm.len);
#endif
	smsg->len = sm.len;
	
	return sbFinalizeMessage(smsg);
}
#undef DO

#define ushortofch2(i0,i8) (unsigned short)((unsigned short)i0 | ((unsigned short)i8 << 8)) 
#define sshortofch2(i0,i8) (signed short)((unsigned short)i0 | ((unsigned short)i8 << 8)) 

#ifndef PIC30
int sbStateDecode(const SBSerialisedMessage *smsg, SBHeliStateRaw* hs)
{
	const unsigned char *msg = smsg->data+SB_MESSAGE_HEADER_LEN;
	hs->errorFlags = msg[0];
	hs->content[1] = ushortofch2(msg[1],msg[2]);
	hs->content[0] = ushortofch2(msg[3],0);
	msg += 4;
	if (TESTCONTENT(hs->content,SBC_MODES)) {
		unsigned short sysmodes = ushortofch2(msg[0],msg[1]);
		unsigned short ctrlmodes = ushortofch2(msg[2],msg[3]);
		msg += fdata[SBC_MODES].datalen*2;
		// printf("Mode: %04X\n",modes);
		hs->mode.navigation = (sysmodes >> SBC_SHIFT_NAV) & SBC_MODE_NAV; 
		hs->mode.communication = (sysmodes >> SBC_SHIFT_COMM) & SBC_MODE_COMM; 
		hs->mode.oavoid = (sysmodes >> SBC_SHIFT_OAVOID) & SBC_MODE_OAVOID; 
		hs->mode.rollAxis = (ctrlmodes >> SBC_SHIFT_ROLL) & SBC_MODE_ROLL; 
		hs->mode.pitchAxis = (ctrlmodes >> SBC_SHIFT_PITCH) & SBC_MODE_PITCH; 
		hs->mode.yawAxis = (ctrlmodes >> SBC_SHIFT_YAW) & SBC_MODE_YAW; 
		hs->mode.altAxis = (ctrlmodes >> SBC_SHIFT_ALT) & SBC_MODE_ALT; 
	}
	if (TESTCONTENT(hs->content,SBC_TIMESTAMP)) {
		unsigned long usl,ush;
		usl = ushortofch2(msg[0],msg[1]);
		ush = ushortofch2(msg[2],msg[3]);
		hs->timeStamp = usl | (ush<<16);
		msg += fdata[SBC_TIMESTAMP].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_RPY)) {
		hs->roll = sshortofch2(msg[0],msg[1]);
		hs->pitch = sshortofch2(msg[2],msg[3]);
		hs->yaw = sshortofch2(msg[4],msg[5]);
		msg+=fdata[SBC_RPY].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_GYRO)) {
		hs->gyro[0] = sshortofch2(msg[0],msg[1]);
		hs->gyro[1] = sshortofch2(msg[2],msg[3]);
		hs->gyro[2] = sshortofch2(msg[4],msg[5]);
		msg+=fdata[SBC_GYRO].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_ACCEL)) {
		hs->accel[0] = sshortofch2(msg[0],msg[1]);
		hs->accel[1] = sshortofch2(msg[2],msg[3]);
		hs->accel[2] = sshortofch2(msg[4],msg[5]);
		msg+=fdata[SBC_ACCEL].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_MAGNETO)) {
		hs->magneto[0] = sshortofch2(msg[0],msg[1]);
		hs->magneto[1] = sshortofch2(msg[2],msg[3]);
		hs->magneto[2] = sshortofch2(msg[4],msg[5]);
		msg+=fdata[SBC_MAGNETO].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_IMUTEMP)) {
		hs->imutemp = ushortofch2(msg[0],msg[1]);
		msg+=fdata[SBC_IMUTEMP].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_ALTITUDE)) {
		hs->zrange = sshortofch2(msg[0],msg[1]);
		hs->zfiltered = sshortofch2(msg[2],msg[3]);
		msg+=fdata[SBC_ALTITUDE].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_PRESSURE)) {
		hs->pressure = sshortofch2(msg[0],msg[1]);
		msg+=fdata[SBC_PRESSURE].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_HRANGES)) {
		hs->hranges[0] = ushortofch2(msg[0],msg[1]);
		hs->hranges[1] = ushortofch2(msg[2],msg[3]);
		hs->hranges[2] = ushortofch2(msg[4],msg[5]);
		hs->hranges[3] = ushortofch2(msg[6],msg[7]);
		msg+=fdata[SBC_HRANGES].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_XY_REL)) {
		hs->xrel = sshortofch2(msg[0],msg[1]);
		hs->yrel = sshortofch2(msg[2],msg[3]);
		msg+=fdata[SBC_XY_REL].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_BATTERY)) {
		hs->battery = ushortofch2(msg[0],msg[1]);
		msg+=fdata[SBC_BATTERY].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_TIMEOUT)) {
		hs->controlTimeout = ushortofch2(msg[0],msg[1]);
		hs->watchdogTimeout = ushortofch2(msg[2],msg[3]);
		msg += fdata[SBC_TIMEOUT].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_O_ATTITUDE)) {
		hs->o_attitude[0] = sshortofch2(msg[0],msg[1]);
		hs->o_attitude[1] = sshortofch2(msg[2],msg[3]);
		hs->o_attitude[2] = sshortofch2(msg[4],msg[5]);
		msg+=fdata[SBC_O_ATTITUDE].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_O_ALTITUDE)) {
		hs->o_altitude = sshortofch2(msg[0],msg[1]);
		msg+=fdata[SBC_O_ALTITUDE].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_O_TOL)) {
		hs->o_tol = sshortofch2(msg[0],msg[1]);
		msg+=fdata[SBC_O_TOL].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_O_XY)) {
		hs->o_xy[0] = sshortofch2(msg[0],msg[1]);
		hs->o_xy[1] = sshortofch2(msg[2],msg[3]);
		msg+=fdata[SBC_O_XY].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_O_OAVOID)) {
		hs->o_oavoid[0] = sshortofch2(msg[0],msg[1]);
		hs->o_oavoid[1] = sshortofch2(msg[2],msg[3]);
		msg+=fdata[SBC_O_OAVOID].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_CHANNELS)) {
		hs->rcChannel[0] = sshortofch2(msg[ 0],msg[ 1]);
		hs->rcChannel[1] = sshortofch2(msg[ 2],msg[ 3]);
		hs->rcChannel[2] = sshortofch2(msg[ 4],msg[ 5]);
		hs->rcChannel[3] = sshortofch2(msg[ 6],msg[ 7]);
		hs->rcChannel[4] = sshortofch2(msg[ 8],msg[ 9]);
		hs->rcChannel[5] = sshortofch2(msg[10],msg[11]);
		hs->rcChannel[6] = sshortofch2(msg[12],msg[13]);
		hs->rcChannel[7] = sshortofch2(msg[14],msg[15]);
		msg+=fdata[SBC_CHANNELS].datalen*2;
	}
	if (TESTCONTENT(hs->content,SBC_COAXSPEED)) {
        hs->coaxspeed.state = msg[0];
        hs->coaxspeed.light = msg[1];
		hs->coaxspeed.vel_x = sshortofch2(msg[2],msg[3]);
		hs->coaxspeed.vel_y = sshortofch2(msg[4],msg[5]);
		msg+=fdata[SBC_CHANNELS].datalen*2;
    }
	return 0;
}
#endif

const char*
sbCtrlModeString(unsigned char c) {
	if (c & SB_CTRL_MANUAL) {
		switch (c & SB_CTRL_MANUAL_MASK) {
			case SB_CTRL_NONE:   return " Man. None ";
			case SB_CTRL_POS:    return " Man. Pos  ";
			case SB_CTRL_VEL:    return " Man. Vel  ";
			case SB_CTRL_REL:    return " Man. Rel  ";
			case SB_CTRL_FORCE:  return " Man. Force";
			default: break;
		}
	} else {
		switch (c & SB_CTRL_MANUAL_MASK) {
			case SB_CTRL_NONE:   return " Auto None ";
			case SB_CTRL_POS:    return " Auto Pos  ";
			case SB_CTRL_VEL:    return " Auto Vel  ";
			case SB_CTRL_REL:    return " Auto Rel  ";
			case SB_CTRL_FORCE:  return " Auto Force";
			default: break;
		}
	}
	return NULL;
}

const char* sbCommModeString(unsigned char c)
{
	switch (c & 0x01) {
		case SB_COM_ONREQUEST: return "On request";
		case SB_COM_CONTINUOUS: return "Continuous";
		default: break;
	}
	return NULL;
}

const char* sbOAModeString(unsigned char c)
{
	switch (c & 0x03) {
		case SB_OA_NONE: return "None";
		case SB_OA_VERTICAL: return "Vertical";
		case SB_OA_HORIZONTAL: return "Horizontal";
		case (SB_OA_HORIZONTAL | SB_OA_VERTICAL): return "Vertical&Horizontal";
		default: break;
	}
	return NULL;
}

const char* sbNavModeString(unsigned char c)
{
	switch (c & 0x07) {
		case SB_NAV_STOP: return "Stop";
		case SB_NAV_IDLE: return "Idle";
		case SB_NAV_TAKEOFF: return "Take Off";
		case SB_NAV_LAND: return "Land";
		case SB_NAV_HOVER: return "Hover";
		case SB_NAV_CTRLLED: return "Controlled";
		case SB_NAV_SINK: return "Sinking";
		case SB_NAV_RAW: return "Raw Control";
		default: return "Incorrect Nav Mode"; 
	}
	return NULL;
}

#ifdef SBC_HAS_IO

void sbStateRawPrint(FILE * fp, SBHeliStateRaw *hs) 
{
	fprintf(fp,"Content: %04X%04X:",hs->content[0], hs->content[1]);
	sbContent16bPrint(fp,hs->content);
	fprintf(fp,"\n");
	fprintf(fp,"Error  : %02X\n",hs->errorFlags);
	if (TESTCONTENT(hs->content,SBC_MODES)) {
		fprintf(fp,"Modes:\n");
		fprintf(fp,"\tNavigation %04X",hs->mode.navigation&SBC_MODE_NAV);
		fprintf(fp,"\tCommunication %04X",hs->mode.communication&SBC_MODE_COMM);
		fprintf(fp,"\tObst. Avoid %04X\n",hs->mode.oavoid&SBC_MODE_OAVOID);
		fprintf(fp,"\tAxis: Roll %04X",hs->mode.rollAxis&SBC_MODE_ROLL);
		fprintf(fp,"\tPitch %04X",hs->mode.pitchAxis&SBC_MODE_PITCH);
		fprintf(fp,"\tYaw %04X",hs->mode.yawAxis&SBC_MODE_YAW);
		fprintf(fp,"\tAlti. %04X",hs->mode.altAxis&SBC_MODE_ALT);
	}
	if (TESTCONTENT(hs->content,SBC_TIMESTAMP)) {
		fprintf(fp,"Timestamp: %04lX\n",hs->timeStamp);
	}
	if (TESTCONTENT(hs->content,SBC_TIMEOUT)) {
		fprintf(fp,"Timeout: control %04X watchdog %04X\n",hs->controlTimeout,hs->watchdogTimeout);
	}
	if (TESTCONTENT(hs->content,SBC_RPY)) {
		fprintf(fp,"RPY: %04X, %04X, %04X\n",hs->roll, hs->pitch, hs->yaw);
	}
	if (TESTCONTENT(hs->content,SBC_GYRO)) {
		fprintf(fp,"Gyro: %04X, %04X, %04X\n",hs->gyro[0], hs->gyro[1], hs->gyro[2]);
	}
	if (TESTCONTENT(hs->content,SBC_ACCEL)) {
		fprintf(fp,"Accel: %04X, %04X, %04X\n",hs->accel[0], hs->accel[1], hs->accel[2]);
	}
	if (TESTCONTENT(hs->content,SBC_MAGNETO)) {
		fprintf(fp,"Magneto: %04X, %04X, %04X\n",hs->magneto[0], hs->magneto[1], hs->magneto[2]);
	}
	if (TESTCONTENT(hs->content,SBC_IMUTEMP)) {
		fprintf(fp,"IMU Temp. %04X\n",hs->imutemp);
	}
	if (TESTCONTENT(hs->content,SBC_ALTITUDE)) {
		fprintf(fp,"Z Range: %04X Filtered: %04X\n",hs->zrange,hs->zfiltered);
	}
	if (TESTCONTENT(hs->content,SBC_PRESSURE)) {
		fprintf(fp,"Pressure: %04X\n",hs->pressure);
	}
	if (TESTCONTENT(hs->content,SBC_HRANGES)) {
		fprintf(fp,"H ranges: %04X, %04X, %04X, %04X\n",
				hs->hranges[0], hs->hranges[1], 
				hs->hranges[2], hs->hranges[3]);
	}
	if (TESTCONTENT(hs->content,SBC_XY_REL)) {
		fprintf(fp,"XY Rel: %04X, %04X\n",hs->xrel, hs->yrel);
	}
	if (TESTCONTENT(hs->content,SBC_BATTERY)) {
		fprintf(fp,"Battery: %04X\n",hs->battery);
	}
	if (TESTCONTENT(hs->content,SBC_TIMEOUT)) {
		fprintf(fp,"Timeouts: %d %d\n",hs->watchdogTimeout,hs->controlTimeout);
	}
	if (TESTCONTENT(hs->content,SBC_O_ATTITUDE)) {
		fprintf(fp,"Output Attitude: %04X, %04X, %04X\n",hs->o_attitude[0], hs->o_attitude[1], hs->o_attitude[2]);
	}
	if (TESTCONTENT(hs->content,SBC_O_ALTITUDE)) {
		fprintf(fp,"Output Altitude: %04X\n", hs->o_altitude);
	}
	if (TESTCONTENT(hs->content,SBC_O_TOL)) {
		fprintf(fp,"Output TOL: %04X\n",hs->o_tol);
	}
	if (TESTCONTENT(hs->content,SBC_O_XY)) {
		fprintf(fp,"Output XY: %04X, %04X\n",hs->o_xy[0], hs->o_xy[1]);
	}
	if (TESTCONTENT(hs->content,SBC_O_OAVOID)) {
		fprintf(fp,"Output OA XY: %04X, %04X\n", hs->o_oavoid[0], hs->o_oavoid[1]);
	}
	if (TESTCONTENT(hs->content,SBC_CHANNELS)) {
		fprintf(fp,"Channels: [ %d %d %d %d %d %d %d %d ]\n", 
				hs->rcChannel[0], hs->rcChannel[1], hs->rcChannel[2],
				hs->rcChannel[3], hs->rcChannel[4], hs->rcChannel[5],
				hs->rcChannel[6], hs->rcChannel[7]);
	}
	if (TESTCONTENT(hs->content,SBC_COAXSPEED)) {
        if (hs->coaxspeed.state & COAXSPEED_AVAILABLE) {
            if (hs->coaxspeed.state & COAXSPEED_VALID_MEASUREMENT) {
                fprintf(fp,"CoaxSpeed: V %4d %4d L %2d%%\n",
                        hs->coaxspeed.vel_x, hs->coaxspeed.vel_y, hs->coaxspeed.light);
            } else {
                fprintf(fp,"CoaxSpeed: V invalid L %2d%%\n", hs->coaxspeed.light);
            }
        } else {
            fprintf(fp,"CoaxSpeed: not detected\n");
        }
	}
}

void sbContent16bPrint(FILE* fp, const unsigned short content[2])
{
	fprintf(fp,"[");
	if (TESTCONTENT(content,SBC_MODES)) {
		fprintf(fp,"MODES,");
	}
	if (TESTCONTENT(content,SBC_TIMESTAMP)) {
		fprintf(fp,"TIMESTAMP,");
	}
	if (TESTCONTENT(content,SBC_RPY)) {
		fprintf(fp,"RPY,");
	}
	if (TESTCONTENT(content,SBC_GYRO)) {
		fprintf(fp,"GYRO,");
	}
	if (TESTCONTENT(content,SBC_ACCEL)) {
		fprintf(fp,"ACCEL,");
	}
	if (TESTCONTENT(content,SBC_MAGNETO)) {
		fprintf(fp,"MAGNETO,");
	}
	if (TESTCONTENT(content,SBC_IMUTEMP)) {
		fprintf(fp,"IMUTEMP,");
	}
	if (TESTCONTENT(content,SBC_ALTITUDE)) {
		fprintf(fp,"ALTITUDE,");
	}
	if (TESTCONTENT(content,SBC_PRESSURE)) {
		fprintf(fp,"PRESSURE,");
	}
	if (TESTCONTENT(content,SBC_TIMEOUT)) {
		fprintf(fp,"TIMEOUT,");
	}
	if (TESTCONTENT(content,SBC_HRANGES)) {
		fprintf(fp,"HRANGES,");
	}
	if (TESTCONTENT(content,SBC_XY_REL)) {
		fprintf(fp,"XY_REL,");
	}
	if (TESTCONTENT(content,SBC_BATTERY)) {
		fprintf(fp,"BATTERY,");
	}
	if (TESTCONTENT(content,SBC_O_ATTITUDE)) {
		fprintf(fp,"O_ATTITUDE,");
	}
	if (TESTCONTENT(content,SBC_O_ALTITUDE)) {
		fprintf(fp,"O_ALTITUDE,");
	}
	if (TESTCONTENT(content,SBC_O_TOL)) {
		fprintf(fp,"O_TOL,");
	}
	if (TESTCONTENT(content,SBC_O_XY)) {
		fprintf(fp,"O_XY,");
	}
	if (TESTCONTENT(content,SBC_O_OAVOID)) {
		fprintf(fp,"O_OAVOID,");
	}
	if (TESTCONTENT(content,SBC_CHANNELS)) {
		fprintf(fp,"CHANNELS,");
	}
	if (TESTCONTENT(content,SBC_COAXSPEED)) {
		fprintf(fp,"COAXSPEED,");
	}
	fprintf(fp,"]");
}


#endif // SBC_HAS_IO

