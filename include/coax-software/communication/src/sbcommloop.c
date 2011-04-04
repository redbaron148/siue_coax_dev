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

#include "com/sbcommloop.h"
#include "com/sbmessage.h"
#include "com/sbversion.h"

#ifdef ERROR
#undef ERROR // Defined in wingdi.h
#endif 
#define ERROR(code,message) {if (comm->error) comm->error(SB_COMMLOOP##code,message);}


void sbCommLoopInit(SBCommLoopSpec *comm, unsigned short controllerVersion)
{
	sbInitialiseVersion(controllerVersion);
	comm->stateReady[0] = comm->stateReady[1] = 0;
	comm->messageToSend[0] = comm->messageToSend[1] = 0;
	comm->cmd_watchdog = 0;
	comm->ctrl_watchdog = 0;
	comm->timeout_cmd = 5000;
	comm->timeout_ctrl = 1000;
	comm->timecount = 0;
	comm->timeinmode = 0;
	comm->landingHeight_mm = 180;
	comm->takeoffHeight_mm = 400;
	comm->landingStep_mm = 10;
	comm->debug_channel = DEFAULT_DEBUG_CHANNEL;
	comm->verbose = 0;
	comm->error = NULL;
	comm->reset = NULL;
	comm->updateControlParams = NULL;
	comm->updateTrimMode = NULL;
	comm->configureBluetooth = NULL;
    comm->setLight = NULL;
}

// TODO:
// - Remove all access control
// - Add one state agenda function per channel
// - Add a number of message in the communication configuration.
void sbCommLoop(SBCommLoopSpec *comm, SBHeliStateRaw *heliState)
{
	static int lastState = -1; 
	long lastTimecount = -1;
	// Initialise loop parameters
	comm->stateReady[0] = comm->stateReady[1] = 0;
	comm->messageToSend[0] = comm->messageToSend[1] = 0;
	comm->cmd_watchdog = 0;
	comm->ctrl_watchdog = 0;
	comm->timeout_cmd = 5000;
	comm->timeout_ctrl = 1000;
	comm->timecount = 0;
	comm->timeinmode = 0;

	// Initialise the global state variable
	memset(heliState,0,sizeof(SBHeliStateRaw));
	heliState->watchdogTimeout = comm->timeout_cmd;
	heliState->controlTimeout = comm->timeout_ctrl;
	heliState->mode.navigation = SB_NAV_STOP;
	heliState->mode.communication = SB_COM_ONREQUEST;
	heliState->mode.oavoid = SB_OA_NONE;
	heliState->mode.rollAxis = SB_CTRL_NONE;
	heliState->mode.pitchAxis = SB_CTRL_NONE;
	heliState->mode.yawAxis = SB_CTRL_NONE;
	heliState->mode.altAxis = SB_CTRL_NONE;
	heliState->setpoint.rollAxis = SB_CTRL_NONE;
	heliState->setpoint.pitchAxis = SB_CTRL_NONE;
	heliState->setpoint.yawAxis = SB_CTRL_NONE;
	heliState->setpoint.altAxis = SB_CTRL_NONE;
	heliState->setpoint.roll = 0;
	heliState->setpoint.pitch = 0;
	heliState->setpoint.yaw = 0;
	heliState->setpoint.altitude = 0;
	heliState->control.roll = 0;
	heliState->control.pitch = 0;
	heliState->control.yaw = 0;
	heliState->control.altitude = 0;
	heliState->commFrequency = 0;

	sbChannelFlush(comm->channel+0);
	sbChannelFlush(comm->channel+1);
	ERROR(_DEBUG,"Comm loop is started\n");
	while (1) {
		SBBasicReplyMessage reply;
		char text[128];
		signed int messageSource;
		unsigned int requestAck = 0;
		SBSerialisedMessage sm;
		SBSerialisedMessage rsm;


#if 1
		if (comm->timecount - lastTimecount >= 1000) {
			lastTimecount = comm->timecount;
			ERROR(_DEBUG,"Is there a message for me?\n");
		}
#endif
		if (heliState->mode.navigation != lastState) {
			sprintf(text,"%s -> %s\n",sbNavModeString(lastState),sbNavModeString(heliState->mode.navigation));
			ERROR(_DEBUG,text);
			lastState = heliState->mode.navigation;
		}

		if (comm->debug_channel != CHANNEL_0_ID) {
			if (comm->stateReady[CHANNEL_0_ID] && comm->messageToSend[CHANNEL_0_ID]) {
				int res;
				ERROR(_DEBUG2,"S0");
				res = sbSendMessage(comm->channel+CHANNEL_0_ID,&comm->serialisedState,SEND_TIMEOUT);
				if (res != 0) {
					ERROR(_DEBUG,"*");
					sbChannelFlush(comm->channel+CHANNEL_0_ID);
				}
				comm->messageToSend[CHANNEL_0_ID] -= 1;
				comm->stateReady[CHANNEL_0_ID] = 0;
				if (comm->messageToSend[CHANNEL_0_ID] == 0) {
					comm->setPeriodicState(CHANNEL_0_ID,0);
				}
			}
		}
		if (comm->debug_channel != CHANNEL_1_ID) {
			if (comm->stateReady[CHANNEL_1_ID] && comm->messageToSend[CHANNEL_1_ID]) {
				int res;
				ERROR(_DEBUG2,"S1");
				res = sbSendMessage(comm->channel+CHANNEL_1_ID,&comm->serialisedState,SEND_TIMEOUT);
				if (res != 0) {
					ERROR(_DEBUG,"*");
					sbChannelFlush(comm->channel+CHANNEL_1_ID);
				}
				comm->messageToSend[CHANNEL_1_ID] -= 1;
				comm->stateReady[CHANNEL_1_ID] = 0;
				if (comm->messageToSend[CHANNEL_1_ID] == 0) {
					comm->setPeriodicState(CHANNEL_1_ID,0);
				}
			}
		}

		messageSource = -1;
		if (comm->debug_channel != CHANNEL_0_ID) {
			if ((messageSource<0) && !sbChannelWaitData(comm->channel+CHANNEL_0_ID,1)) {
				ERROR(_DEBUG2,"There is data UART\n");
				if (sbWaitRawMessage(comm->channel+CHANNEL_0_ID,-1, &sm, RECEIVE_TIMEOUT)) {
					// this should not fail, but just in case, discard anything 
					// in the message queue
					sbChannelFlush(comm->channel+CHANNEL_0_ID);
					ERROR(_WARNING,"Failed to read message\n");
				} else {
					ERROR(_DEBUG2,"I got the message\n");
					messageSource = CHANNEL_0_ID;
				}
			}
		}
		if (comm->debug_channel != CHANNEL_1_ID) {
			if ((messageSource<0) && !sbChannelWaitData(comm->channel+CHANNEL_1_ID,1)) {
				ERROR(_DEBUG2,"There is data on BT\n");
				if (sbWaitRawMessage(comm->channel+CHANNEL_1_ID,-1, &sm, RECEIVE_TIMEOUT)) {
					// this should not fail, but just in case, discard anything 
					// in the message queue
					sbChannelFlush(comm->channel+CHANNEL_1_ID);
					ERROR(_WARNING,"Failed to read message\n");
				} else {
					ERROR(_DEBUG2,"I got the message\n");
					messageSource = CHANNEL_1_ID;
				}
			}
		}

		if (messageSource<0) {
			continue;
#if 1
		} else {
			//sprintf(text,"Received message %02X (%d) from %02X\n\r",sm.msgid,sm.msgid,messageSource);
			sprintf(text,"[%02X]",sm.msgid);
			ERROR(_DEBUG,text);
#endif
		}

#if 1
		requestAck = (sm.msgid & SB_MSGID_REQACK);
		sm.msgid = sm.msgid & SB_MSGID_MASK;
		reply.status = SB_REPLY_OK;
		switch (sm.msgid) {
			/*** Function allowed from any source ***/
			case SB_MSGID_CFG_COMMLOOP:
				{
					SBConfigureCommLoop col;
					if (sbCfgCommLoopDecode(&sm,&col)) {
						ERROR(_ERROR,"CFG_COMMLOOP: sbCfgCommLoopDecode\n");
						reply.status = SB_REPLY_DECODE_FAILURE;
					} else {
#if 0
						if (col.debug_channel & 0xFE) {
							comm->debug_channel = 0xFF;
							comm->verbose = 0; // no debug channel, no need to bother with sprinf
						} else {
							comm->debug_channel = col.debug_channel;
							comm->verbose = col.verbosity;
						}
#else
						// Channel selection is ignored for now
						comm->verbose = col.verbosity;
#endif
						reply.status = SB_REPLY_OK;
					}
					if (comm->verbose) {
						ERROR(_DEBUG,"Set CommLoop Verbosity\n");
					} else {
						ERROR(_WARNING,"Stopped CommLoop Verbosity\n");
					}
					if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
						ERROR(_ERROR,"CFG_COMMLOOP: sbBasicReplyEncode\n");
					} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
						ERROR(_ERROR,"CFG_COMMLOOP: sbSendMessage");
					}
				}
				break;
			case SB_MSGID_STRING:
				{
					SBStringMessage s;
					if (sbStringMsgDecode(&sm,&s)) {
						// Ignore...? 
					} else {
						// just in case
						s.text[SB_STRING_MESSAGE_LENGTH-1] = 0;
						ERROR(_DEBUG,(const char*)s.text);
					}
				}
				break;
			case SB_MSGID_KEEP_ALIVE:
				{
					comm->cmd_watchdog = 0;
					if (requestAck) {
						reply.status = SB_REPLY_OK;
						if (sbBasicReplyEncode(&rsm,0,
									sm.msgid|SB_MSGID_REPLY,&reply)) {
							ERROR(_ERROR,"KEEP_ALIVE: sbBasicReplyEncode");
						} else if(sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
							ERROR(_ERROR,"KEEP_ALIVE: sbSendMessage");
						}
					}
					ERROR(_DEBUG,"Keep Alive\n");
				}
				break;
			case SB_MSGID_CFG_COMM:
				{
					SBConfigureCommunication com;
					if (sbCfgCommDecode(&sm,&com)) {
						ERROR(_ERROR,"CFG_COMM: sbCfgCommDecode");
						reply.status = SB_REPLY_DECODE_FAILURE;
					} else {
						heliState->mode.communication = com.commMode & 0x01;
						if (heliState->mode.communication == SB_COM_CONTINUOUS) {
							unsigned int period;
							heliState->content[0] = com.content[0] & comm->capacities[0];
							heliState->content[1] = com.content[1] & comm->capacities[1];
							if (com.frequency == 0) com.frequency = 1;
							heliState->commFrequency = com.frequency;
							period = 1000/com.frequency;
							comm->messageToSend[messageSource] = com.numMessages;
							comm->setPeriodicState(messageSource,period);
							reply.status = SB_REPLY_OK;
							ERROR(_DEBUG,"Started continuous communication\n");
						} else {
							comm->messageToSend[messageSource] = 0; 
							heliState->commFrequency = 0;
							comm->setPeriodicState(messageSource,0);
							reply.status = SB_REPLY_OK;
							ERROR(_DEBUG,"Started on-demand communication\n");
						}
					}
					if (sbBasicReplyEncode(&rsm,0,
								sm.msgid|SB_MSGID_REPLY,&reply)) {
						ERROR(_ERROR,"CFG_COMM: sbBasicReplyEncode");
					} else if(sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
						ERROR(_ERROR,"CFG_COMM: sbSendMessage");
					}
				}
				break;
			case SB_MSGID_STATE:
				{
					SBRequestState rs;
					if (sbReqStateDecode(&sm,&rs)) {
						ERROR(_ERROR,"STATE: sbReqStateDecode");
					} else {
						SBHeliStateRaw localState;
						rs.content[0] = rs.content[0] & comm->capacities[0];
						rs.content[1] = rs.content[1] & comm->capacities[1];
						comm->updateHeliState();
						memcpy(&localState,heliState,sizeof(SBHeliStateRaw));
						localState.content[0] = rs.content[0];
						localState.content[1] = rs.content[1];
						// ERROR(_DEBUG,"Encoding request\n");
						if (sbStateEncode(&rsm,0,&localState)) {
							ERROR(_ERROR,"STATE: Failed to encode heli state\n");
						} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
							ERROR(_ERROR,"STATE: sbSendMessage");
						}
						sprintf(text,"Accepted state request %04X %04X\n",
								rs.content[0],rs.content[1]);
						ERROR(_DEBUG,text);
					}
				}
				break;
			case SB_MSGID_GET_VERSION:
				{
					const SBVersionStatus *version = sbGetCompiledVersion();
					if (sbVersionMsgEncode(&rsm,0,version)) {
						ERROR(_ERROR,"GET_VERSION: sbVersionMsgEncode\n");
					} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
						ERROR(_ERROR,"GET_VERSION: sbSendMessage");
					}
					ERROR(_DEBUG,"Received version request\n");
				}
				break;
			case SB_MSGID_SENSORLIST:
				{
					SBSensorListMessage sl;
					sl.content[0] = comm->capacities[0];
					sl.content[1] = comm->capacities[1];
					if (sbSensorListEncode(&rsm,0,&sl)) {
						ERROR(_ERROR,"SENSORLIST: sbSensorListEncode\n");
					} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
						ERROR(_ERROR,"SENSORLIST: sbSendMessage");
					}
					ERROR(_DEBUG,"Received sensor list request\n");
				}
				break;
				/*** Connection function ***/
			case SB_MSGID_CONNECT:
				{
					reply.status = SB_REPLY_OK;
					comm->cmd_watchdog = 0;
					ERROR(_DEBUG,"Connection accepted\n");
					if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
						ERROR(_ERROR,"CONNECT: sbBasicReplyEncode\n");
					} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
						ERROR(_ERROR,"CONNECT: sbSendMessage");
					}
				}
				break;
				/*** Functions allowed only from the active Controller ***/
			case SB_MSGID_DISCONNECT:
				{
					reply.status = SB_REPLY_OK;

					if (sbBasicReplyEncode(&rsm,0,
								sm.msgid|SB_MSGID_REPLY,&reply)) {
						ERROR(_ERROR,"DISCONNECT: sbBasicReplyEncode\n");
					} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
						ERROR(_ERROR,"DISCONNECT: sbSendMessage");
					}
					ERROR(_DEBUG,"Disconnection accepted\n");
				}
				break;
			case SB_MSGID_RESET:
				{
					reply.status = SB_REPLY_OK;
					if (sbBasicReplyEncode(&rsm,0,
								sm.msgid|SB_MSGID_REPLY,&reply)) {
						ERROR(_ERROR,"RESET: sbBasicReplyEncode\n");
					} else {
						sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT);
						// TODO: error management
					}
					ERROR(_DEBUG,"RESET Received\n");
#if 1
					if (comm->reset) comm->reset();
					while (1) {};
#endif
				}
				break;
			case SB_MSGID_CFG_BLUETOOTH:
				{
					SBConfigureBluetoothMessage cm;
					if (sbConfigureBluetoothDecode(&sm,&cm)) {
						ERROR(_ERROR,"CFG_BLUETOOTH: sbConfigureBluetoothDecode\n");
						reply.status = SB_REPLY_DECODE_FAILURE;
					} else if (comm->configureBluetooth) {
						int res = comm->configureBluetooth(cm.code,cm.name);
						reply.status = (res==0)?SB_REPLY_OK:SB_REPLY_ERROR;
					} else {
						reply.status = SB_REPLY_OK;
					}
					if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
						ERROR(_ERROR,"CFG_BLUETOOTH: sbBasicReplyEncode\n");
					} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
						ERROR(_ERROR,"CFG_BLUETOOTH: sbSendMessage");
					}
					ERROR(_DEBUG,"Configured Bluetooth (suggest reset)\n");
				}
				break;
			case SB_MSGID_SET_LIGHT:
				{
					SBCoaxSpeedSetLight sl;
					if (sbCoaxSpeedSetLightDecode(&sm,&sl)) {
						ERROR(_ERROR,"SET_LIGHT: sbCoaxSpeedSetLightDecode\n");
						reply.status = SB_REPLY_DECODE_FAILURE;
					} else {
                        if (comm->setLight) {
                            comm->setLight(sl.percent);
                        }
						reply.status = SB_REPLY_OK;
					}
					if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
						ERROR(_ERROR,"SET_LIGHT: sbBasicReplyEncode\n");
					} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
						ERROR(_ERROR,"SET_LIGHT: sbSendMessage");
					}
					ERROR(_DEBUG,"Set CoaxSpeed Light\n");
				}
				break;
			case SB_MSGID_CFG_OAVOID:
				{
					SBConfigureObstAvoid coa;
					if (sbCfgOADecode(&sm,&coa)) {
						ERROR(_ERROR,"CFG_OAVOID: sbCfgOADecode\n");
						reply.status = SB_REPLY_DECODE_FAILURE;
					} else {
						heliState->mode.oavoid = coa.oavoidMode & 0x03;
						reply.status = SB_REPLY_OK;
					}
					if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
						ERROR(_ERROR,"CFG_OAVOID: sbBasicReplyEncode\n");
					} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
						ERROR(_ERROR,"CFG_OAVOID: sbSendMessage");
					}
					ERROR(_DEBUG,"Configured Obstacle Avoidance\n");
				}
				break;
			case SB_MSGID_CFG_TIMEOUT:
				{
					SBConfigureTimeout cot;
					if (sbCfgTimeoutDecode(&sm,&cot)) {
						ERROR(_ERROR,"CFG_TIMEOUT: sbCfgTimeoutDecode\n");
						reply.status = SB_REPLY_DECODE_FAILURE;
					} else {
						// a timeout of zero is ignored
						if (cot.controlTimeout) {
							comm->timeout_ctrl = cot.controlTimeout;
						}
						if (cot.watchdogTimeout) {
							comm->timeout_cmd = cot.watchdogTimeout;
						}
						heliState->watchdogTimeout = comm->timeout_cmd;
						heliState->controlTimeout = comm->timeout_ctrl;
						reply.status = SB_REPLY_OK;
						sprintf(text,"Configured Timeout: %d & %d\n",
								heliState->controlTimeout,heliState->watchdogTimeout);
						ERROR(_DEBUG,text);
						comm->cmd_watchdog = 0;
						comm->ctrl_watchdog = 0;
					}
					if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
						ERROR(_ERROR,"CFG_TIMEOUT: sbBasicReplyEncode\n");
					} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
						ERROR(_ERROR,"CFG_TIMEOUT: sbSendMessage");
					}
				}
				break;
			case SB_MSGID_TRIMMODE:
				{
					SBTrimModeMessage msg;
					if (!comm->updateTrimMode) {
						ERROR(_WARNING,"Ignored Trim Mode (No handler)\n");
						break;
					}
					if (sm.len == 0) {
						// Get message
						comm->updateTrimMode(0,&msg);
						if (sbTrimModeEncode(&rsm,0,&msg,0)) {
							ERROR(_ERROR,"TRIM_MODE: sbTrimModeEncode\n");
						} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
							ERROR(_ERROR,"TRIM_MODE: sbSendMessage");
						}
					} else {
						// Set message
						if (sbTrimModeDecode(&sm,&msg)) {
							ERROR(_ERROR,"TRIM_MODE: sbTrimModeDecode\n");
							reply.status = SB_REPLY_DECODE_FAILURE;
						} else if (comm->updateTrimMode(1,&msg)) {
							reply.status = SB_REPLY_ERROR;
							ERROR(_WARNING,"Ignored Trim Mode\n");
						} else {
							reply.status = SB_REPLY_OK;
							ERROR(_DEBUG,"Configured Trim Mode\n");
						}
						if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
							ERROR(_ERROR,"TRIM_MODE: sbBasicReplyEncode\n");
						} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
							ERROR(_ERROR,"TRIM_MODE: sbSendMessage");
						}
					}
				}
				break;
			case SB_MSGID_CUSTOM:
				{
					SBCustomMessage msg;
					if (sm.len == 0) {
						// Get message
						if (sbCustomMsgEncode(&rsm,0,&comm->customState,1)) {
							ERROR(_ERROR,"CUSTOM: sbCustomMsgEncode\n");
						} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
							ERROR(_ERROR,"CUSTOM: sbSendMessage");
						}
					} else {
						// Set message
						if (sbCustomMsgDecode(&sm,&msg)) {
							ERROR(_ERROR,"CUSTOM: sbCustomMsgDecode\n");
							reply.status = SB_REPLY_DECODE_FAILURE;
						} else {
                            memcpy(&comm->customState,&msg,sizeof(SBCustomMessage));
							reply.status = SB_REPLY_OK;
							ERROR(_DEBUG,"CUSTOM: Set custom message\n");
						}
						if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
							ERROR(_ERROR,"CUSTOM: sbBasicReplyEncode\n");
						} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
							ERROR(_ERROR,"CUSTOM: sbSendMessage");
						}
					}
				}
				break;
			case SB_MSGID_CTRLPARM:
				{
					SBControlParametersMessage msg;
					if (!comm->updateControlParams) {
						ERROR(_WARNING,"Ignored Control Parameters (No handler)\n");
						break;
					}
					if (sm.len == 0) {
						// Get message
						comm->updateControlParams(0,&msg);
						if (sbCtrlParametersEncode(&rsm,0,&msg,0)) {
							ERROR(_ERROR,"CTRL_PARM: sbTrimModeEncode\n");
						} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
							ERROR(_ERROR,"CTRL_PARM: sbSendMessage");
						}
					} else {
						// Set message
						if (sbCtrlParametersDecode(&sm,&msg)) {
							ERROR(_ERROR,"CTRL_PARM: sbCtrlParametersDecode\n");
							reply.status = SB_REPLY_DECODE_FAILURE;
						} else if (comm->updateControlParams(1,&msg)) {
							reply.status = SB_REPLY_ERROR;
							ERROR(_WARNING,"Ignored Control Parameters\n");
						} else {
							reply.status = SB_REPLY_OK;
							ERROR(_DEBUG,"Configured Control Parameters\n");
						}
						if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
							ERROR(_ERROR,"CTRL_PARM: sbBasicReplyEncode\n");
						} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
							ERROR(_ERROR,"CTRL_PARM: sbSendMessage");
						}
					}
				}
				break;
			case SB_MSGID_CFG_RAW_CMD:
				{
					SBConfigureRawControl cot;
					if (sbCfgRawControlDecode(&sm,&cot)) {
						ERROR(_ERROR,"CFG_RAW_CMD: sbCfgRawControlDecode\n");
						reply.status = SB_REPLY_DECODE_FAILURE;
					} else {
						heliState->rawSpeedProfile[0] = cot.speedprofile1;
						heliState->rawSpeedProfile[1] = cot.speedprofile2;
						reply.status = SB_REPLY_OK;
						ERROR(_DEBUG,"Configured Raw Command\n");
					}
					if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
						ERROR(_ERROR,"CFG_RAW_CMD: sbBasicReplyEncode\n");
					} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
						ERROR(_ERROR,"CFG_RAW_CMD: sbSendMessage");
					}
				}
				break;
			case SB_MSGID_CFG_CONTROL:
				{
					SBConfigureControl cot;
					if (sbCfgControlDecode(&sm,&cot)) {
						ERROR(_ERROR,"CFG_CONTROL: sbCfgControlDecode\n");
						reply.status = SB_REPLY_DECODE_FAILURE;
					} else {
						heliState->setpoint.rollAxis = cot.roll;
						heliState->setpoint.pitchAxis = cot.pitch;
						heliState->setpoint.yawAxis = cot.yaw;
						heliState->setpoint.altAxis = cot.altitude;
						reply.status = SB_REPLY_OK;
						ERROR(_DEBUG,"Configured Control Command\n");
#if 0
						printf("%s %s %s %s\n",
								sbCtrlModeString(heliState->setpoint.rollAxis),
								sbCtrlModeString(heliState->setpoint.pitchAxis),
								sbCtrlModeString(heliState->setpoint.yawAxis),
								sbCtrlModeString(heliState->setpoint.altAxis));
#endif
					}
					if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
						ERROR(_ERROR,"CFG_CONTROL: sbBasicReplyEncode\n");
					} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
						ERROR(_ERROR,"CFG_CONTROL: sbSendMessage");
					}
				}
				break;
			case SB_MSGID_SET_CONTROL:
				{
					SBSetControl sc;
					if (heliState->mode.navigation == SB_NAV_CTRLLED) {
						if (sbSetControlDecode(&sm,&sc)) {
							ERROR(_ERROR,"SET_CONTROL: sbSetControlDecode\n");
							reply.status = SB_REPLY_DECODE_FAILURE;
						} else {
							heliState->setpoint.roll = sc.roll;
							heliState->setpoint.pitch = sc.pitch;
							heliState->setpoint.yaw = sc.yaw;
							heliState->setpoint.altitude = sc.altitude;
							comm->ctrl_watchdog = 0;
							reply.status = SB_REPLY_OK;
							ERROR(_DEBUG,"Accepted Control Command\n");
						}
					} else {
						reply.status = SB_REPLY_INVALID_NAVMODE;
					}
					comm->cmd_watchdog = 0;
					if (requestAck) {
						if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
							ERROR(_ERROR,"SET_CONTROL: sbBasicReplyEncode\n");
						} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
							ERROR(_ERROR,"SET_CONTROL: sbSendMessage");
						}
					} else {
						// TODO?
					}
				}
				break;
			case SB_MSGID_SET_CONTROL_WITH_TIMESTAMP:
				{
					SBSetControlWithTimestamp sc;
					if (heliState->mode.navigation == SB_NAV_CTRLLED) {
						if (sbSetControlWithTimestampDecode(&sm,&sc)) {
							ERROR(_ERROR,"SET_CONTROL: sbSetControlWithTimestampDecode\n");
							reply.status = SB_REPLY_DECODE_FAILURE;
						} else {
							heliState->setpoint.roll = sc.roll;
							heliState->setpoint.pitch = sc.pitch;
							heliState->setpoint.yaw = sc.yaw;
							heliState->setpoint.altitude = sc.altitude;
							comm->ctrl_watchdog = 0;
							reply.status = SB_REPLY_OK;
							ERROR(_DEBUG,"Accepted Control Command\n");
						}
					} else {
						reply.status = SB_REPLY_INVALID_NAVMODE;
					}
					comm->cmd_watchdog = 0;
					if (requestAck) {
						if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
							ERROR(_ERROR,"SET_CONTROL: sbBasicReplyEncode\n");
						} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
							ERROR(_ERROR,"SET_CONTROL: sbSendMessage");
						}
					} else {
						// TODO?
					}
				}
				break;
			case SB_MSGID_RAW_COMMAND:
				{
					SBRawControl sc;
					if (heliState->mode.navigation == SB_NAV_RAW) {
						if (sbRawControlDecode(&sm,&sc)) {
							ERROR(_ERROR,"RAW_COMMAND: sbRawControlDecode\n");
							reply.status = SB_REPLY_DECODE_FAILURE;
						} else {
							heliState->setpoint.motor1 = sc.motor1;
							heliState->setpoint.motor2 = sc.motor2;
							heliState->setpoint.servo1 = sc.servo1;
							heliState->setpoint.servo2 = sc.servo2;
							comm->ctrl_watchdog = 0;
							reply.status = SB_REPLY_OK;
							ERROR(_DEBUG,"Accepted Raw Command\n");
						}
					} else {
						reply.status = SB_REPLY_INVALID_NAVMODE;
					}
					comm->cmd_watchdog = 0;
					if (requestAck) {
						if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
							ERROR(_ERROR,"RAW_COMMAND: sbBasicReplyEncode\n");
						} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
							ERROR(_ERROR,"RAW_COMMAND: sbSendMessage");
						}
					} else {
						// TODO?
					}
				}
				break;
			case SB_MSGID_SET_NAVMODE:
				{
					SBSetNavigationMode nm;
					unsigned char navMode = heliState->mode.navigation;
					reply.status = SB_REPLY_OK;
					if (sbSetNavDecode(&sm,&nm)) {
						ERROR(_ERROR,"SET_NAVMODE: sbSetNavDecode\n");
						reply.status = SB_REPLY_DECODE_FAILURE;
					} else if (nm.mode != navMode) {
						switch (nm.mode) {
							case SB_NAV_STOP:
								if ((navMode != SB_NAV_IDLE) && (navMode != SB_NAV_RAW)) {
									reply.status = SB_REPLY_INVALID_NAVMODE;
								}
								break;
							case SB_NAV_IDLE:
								if (navMode != SB_NAV_STOP) reply.status = SB_REPLY_INVALID_NAVMODE;
								break;
							case SB_NAV_RAW:
								if (navMode != SB_NAV_STOP) reply.status = SB_REPLY_INVALID_NAVMODE;
								heliState->setpoint.motor1 = 0;
								heliState->setpoint.motor2 = 0;
								heliState->setpoint.servo1 = 0;
								heliState->setpoint.servo2 = 0;
								break;
							case SB_NAV_TAKEOFF:
								switch (navMode) {
									case SB_NAV_IDLE:
									case SB_NAV_HOVER:
										break;
									default:
										reply.status = SB_REPLY_INVALID_NAVMODE;
								}
								if (comm->timeinmode < MIN_TIME_IN_IDLE) reply.status = SB_REPLY_TOO_EARLY;
								break;
							case SB_NAV_LAND:
								switch (navMode) {
									case SB_NAV_IDLE:
									case SB_NAV_HOVER:
									case SB_NAV_TAKEOFF:
									case SB_NAV_RAW:
									case SB_NAV_SINK:
									case SB_NAV_CTRLLED:
										break;
									default:
										reply.status = SB_REPLY_INVALID_NAVMODE;
								}
								break;
							case SB_NAV_HOVER:
								if ((navMode != SB_NAV_SINK) && (navMode != SB_NAV_CTRLLED)) {
									reply.status = SB_REPLY_INVALID_NAVMODE;
								}
								break;
							case SB_NAV_CTRLLED:
								if ((navMode != SB_NAV_SINK) && (navMode != SB_NAV_HOVER)) {
									reply.status = SB_REPLY_INVALID_NAVMODE;
								} else {
									// Sensible values when switching modes
									heliState->setpoint.roll = 0;
									heliState->setpoint.pitch = 0;
									heliState->setpoint.yaw = 0;
									heliState->setpoint.altitude = 0;
									if (heliState->setpoint.yawAxis == SB_CTRL_POS) {
										heliState->setpoint.yaw = heliState->yaw;
									}
									if (heliState->setpoint.altAxis == SB_CTRL_REL) {
										heliState->setpoint.altitude = heliState->zrange;
									}
									comm->ctrl_watchdog = 0;
								}
								break;
							case SB_NAV_SINK:
							default :
								// this is not a mode one can ask
								reply.status = SB_REPLY_INVALID_NAVMODE;
								break;
						}
					} else {
						// no change requested
						reply.status = SB_REPLY_OK;
					}
					comm->cmd_watchdog = 0;
					if (requestAck) {
						if (sbBasicReplyEncode(&rsm,0, sm.msgid|SB_MSGID_REPLY,&reply)) {
							ERROR(_ERROR,"SET_NAVMODE: sbBasicReplyEncode\n");
						} else if (sbSendMessage(comm->channel+messageSource,&rsm,SEND_TIMEOUT)) {
							ERROR(_ERROR,"SET_NAVMODE: sbSendMessage");
						}
					} else {
						// TODO?
					}
					if (reply.status == SB_REPLY_OK) {
						heliState->mode.navigation = nm.mode;
						sprintf(text,"Accepted Nav Mode %s\n",sbNavModeString(nm.mode));
						ERROR(_DEBUG,text);
					} else {
						sprintf(text,"Ignored Nav Mode %s\n",sbNavModeString(nm.mode));
						ERROR(_DEBUG,text);
					}
				}
				break;
			default:
				{
					ERROR(_ERROR,"Unknown command\n");

					reply.status = SB_REPLY_UNKNOWN;
					if (sbBasicReplyEncode(&rsm,0,
								sm.msgid|SB_MSGID_REPLY,&reply)) {
						ERROR(_ERROR,"Unknown command: sbBasicReplyEncode\n");
					} else if (sbSendMessage(comm->channel+messageSource,&rsm,500)) {
						ERROR(_ERROR,"SB_XXXX: sbSendMessage");
					}
					sprintf(text,"\n\rUnhandled message <%02X>\n",sm.msgid);
					ERROR(_DEBUG,text);
				}
				break;
		}
		if (reply.status != SB_REPLY_OK) {
			sprintf(text,"Msg %02X: Error code %d\n",sm.msgid,reply.status);
			ERROR(_WARNING,text);
		} else {
			sprintf(text,"Msg %02X: success\n",sm.msgid);
			ERROR(_DEBUG,text);
		}

#endif
	}
}

// #define DEBUG_SONAR

void sbStateMachine(SBCommLoopSpec *comm, SBHeliStateRaw *heliState)
{
	unsigned int currentmode;
	int oapitch=0, oaroll=0, oaalt=-1;
	comm->updateHeliState();
	currentmode = heliState->mode.navigation;
	//
	// TODO: Implement state machine here
	// First see if the state needs to be changed
	switch (heliState->mode.navigation) {
		case SB_NAV_IDLE:
			if ((comm->timeout_cmd!=0xFFFF) && (comm->cmd_watchdog >= comm->timeout_cmd)) {
				heliState->mode.navigation = SB_NAV_STOP;
				// ERROR(_DEBUG,"IDLE -> STOP\n");
			}
			break;
		case SB_NAV_SINK:
		case SB_NAV_LAND:
			comm->cmd_watchdog = 0; // watchdog is meaning less while we try to land.
#ifndef DEBUG_SONAR
			if (heliState->zrange <= comm->landingHeight_mm) {
#endif
				heliState->mode.navigation = SB_NAV_IDLE;
				// ERROR(_DEBUG,"SINK/LAND -> IDLE\n");
#ifndef DEBUG_SONAR
			}
#endif
			break;
		case SB_NAV_TAKEOFF:
			// printf("TAKEOFF %d : %d\n",comm->takeoffHeight_mm-20,heliState->zrange);
			comm->cmd_watchdog = 0; // watchdog is meaning less while we takeoff
#ifndef DEBUG_SONAR
			if (heliState->zrange > (comm->takeoffHeight_mm-50)) {
#endif
				heliState->setpoint.altAxis = SB_CTRL_REL;
				heliState->setpoint.altitude = heliState->zrange;
				heliState->mode.navigation = SB_NAV_HOVER;
				comm->cmd_watchdog = 0;
				// ERROR(_DEBUG,"TAKEOFF -> HOVER\n");
#ifndef DEBUG_SONAR
			}
#endif
			break;
		case SB_NAV_RAW:
			if ((comm->timeout_cmd!=0xFFFF) && (comm->cmd_watchdog >= comm->timeout_cmd)) {
				heliState->mode.navigation = SB_NAV_STOP;
				// ERROR(_DEBUG,"RAW -> STOP\n");
			} else if ((comm->timeout_ctrl!=0xFFFF) && (comm->ctrl_watchdog >= comm->timeout_ctrl)) {
				heliState->mode.navigation = SB_NAV_STOP;
				// ERROR(_DEBUG,"RAW -> HOVER\n");
			}
			break;
		case SB_NAV_STOP:
			break;
		case SB_NAV_HOVER:
			if ((comm->timeout_cmd!=0xFFFF) && (comm->cmd_watchdog >= comm->timeout_cmd)) {
				heliState->mode.navigation = SB_NAV_SINK;
				// ERROR(_DEBUG,"HOVER -> SINK\n");
			}
			break;
		case SB_NAV_CTRLLED:
			if ((comm->timeout_cmd!=0xFFFF) && (comm->cmd_watchdog >= comm->timeout_cmd)) {
				heliState->mode.navigation = SB_NAV_SINK;
				// ERROR(_DEBUG,"CTRLLED -> SINK\n");
			} else if ((comm->timeout_ctrl!=0xFFFF) && (comm->ctrl_watchdog >= comm->timeout_ctrl)) {
				heliState->mode.navigation = SB_NAV_HOVER;
				heliState->setpoint.altitude = heliState->zrange;
				// ERROR(_DEBUG,"CTRLLED -> HOVER\n");
			}
			break;
		default:
			break;
	}
	if (heliState->mode.navigation != currentmode) {
		comm->timeinmode = 0;
	}

	// Then take into account the obstacle avoidance state
	if (heliState->mode.oavoid & SB_OA_VERTICAL) {
		int zrange = heliState->zrange;
		if ((zrange>0) && (zrange < SB_OA_MAX_ALT)) {
			oaalt = SB_OA_MAX_ALT;
		} else {
			oaalt = -1;
		}
#if 0
		printf("OA V: D %d C %d\n",zrange,oaalt); 
#endif
	} else {
		oaalt = -1;
	}
	
	if (heliState->mode.oavoid & SB_OA_HORIZONTAL) {
		int rfront=0,rleft=0,rright=0,rback=0;
		rfront = heliState->hranges[SB_RANGE_FRONT];
		rleft = heliState->hranges[SB_RANGE_LEFT];
		rright = heliState->hranges[SB_RANGE_RIGHT];
		rback = heliState->hranges[SB_RANGE_BACK];
		if ((rfront>0) && (rfront < SB_OA_MAX_DIST)) {
			oapitch += ((SB_OA_MAX_DIST-rfront) * SB_OA_MAX_CORRECTION)/SB_OA_MAX_DIST;
		}
		if ((rback>0) && (rback < SB_OA_MAX_DIST)) {
			oapitch -= ((SB_OA_MAX_DIST-rback) * SB_OA_MAX_CORRECTION)/SB_OA_MAX_DIST;
		}
		if ((rleft>0) && (rleft < SB_OA_MAX_DIST)) {
			oaroll -= ((SB_OA_MAX_DIST-rleft) * SB_OA_MAX_CORRECTION)/SB_OA_MAX_DIST;
		}
		if ((rright>0) && (rright < SB_OA_MAX_DIST)) {
			oaroll += ((SB_OA_MAX_DIST-rright) * SB_OA_MAX_CORRECTION)/SB_OA_MAX_DIST;
		}
#if 0
		printf("OA H: D %d %d %d %d C %d %d S %d %d\n",
				rfront,rleft,rright,rback,oaroll,oapitch,
				heliState->setpoint.roll,
				heliState->setpoint.pitch);
#endif
	} else {
		// Just to be sure!
		oaroll = 0;
		oapitch = 0;
	}
	
	// Then take a decision based on the state
	// printf("Nav mode: %s\n",sbNavModeString(heliState->mode.navigation));
	switch (heliState->mode.navigation) {
		case SB_NAV_RAW:
			heliState->mode.altAxis = SB_CTRL_NONE;
			heliState->mode.rollAxis = SB_CTRL_NONE;
			heliState->mode.pitchAxis = SB_CTRL_NONE;
			heliState->mode.yawAxis = SB_CTRL_NONE;
			break;
		case SB_NAV_IDLE:
		case SB_NAV_STOP:
			heliState->mode.altAxis = SB_CTRL_NONE;
			heliState->mode.rollAxis = SB_CTRL_NONE;
			heliState->mode.pitchAxis = SB_CTRL_NONE;
			heliState->mode.yawAxis = SB_CTRL_NONE;
			break;
		case SB_NAV_LAND:
			heliState->mode.altAxis = SB_CTRL_REL;
			heliState->mode.rollAxis = SB_CTRL_POS;
			heliState->mode.pitchAxis = SB_CTRL_POS;
			heliState->mode.yawAxis = SB_CTRL_VEL;
			heliState->control.altitude = heliState->zrange - comm->landingStep_mm; 
			heliState->control.roll = 0; 
			heliState->control.pitch = 0; 
			heliState->control.yaw = 0; 
			break;
		case SB_NAV_SINK:
			heliState->mode.altAxis = SB_CTRL_REL;
			heliState->mode.rollAxis = SB_CTRL_POS;
			heliState->mode.pitchAxis = SB_CTRL_POS;
			heliState->mode.yawAxis = SB_CTRL_VEL;
			heliState->control.altitude = heliState->zrange - comm->landingStep_mm; 
			heliState->control.roll = oaroll; 
			heliState->control.pitch = oapitch; 
			heliState->control.yaw = 0; 
			break;
		case SB_NAV_TAKEOFF:
			heliState->mode.altAxis = SB_CTRL_REL;
			heliState->mode.rollAxis = SB_CTRL_POS;
			heliState->mode.pitchAxis = SB_CTRL_POS;
			heliState->mode.yawAxis = SB_CTRL_VEL;
			heliState->control.altitude = comm->takeoffHeight_mm;
			heliState->control.roll = 0; 
			heliState->control.pitch = 0; 
			heliState->control.yaw = 0; 
			break;
		case SB_NAV_HOVER:
			heliState->mode.rollAxis = SB_CTRL_POS;
			heliState->mode.pitchAxis = SB_CTRL_POS;
			heliState->mode.yawAxis = SB_CTRL_VEL;
			if (oaalt>0) {
				heliState->mode.altAxis = SB_CTRL_REL;
				heliState->control.altitude = oaalt;
			} else {
				heliState->mode.altAxis = SB_CTRL_REL;
				heliState->control.altitude = heliState->setpoint.altitude; 
			}

			heliState->control.roll = oaroll; 
			heliState->control.pitch = oapitch; 
			heliState->control.yaw = 0; 
			// Zstar specified at transition  
			break;
		case SB_NAV_CTRLLED:
			// printf("Controlled: alt %s\n",sbCtrlModeString(heliState->setpoint.altAxis));
			heliState->mode.yawAxis = heliState->setpoint.yawAxis;
			heliState->control.yaw = heliState->setpoint.yaw;

			if (oaalt>0) {
				heliState->mode.altAxis = SB_CTRL_REL;
				heliState->control.altitude = oaalt;
			} else {
				heliState->mode.altAxis = heliState->setpoint.altAxis;
				heliState->control.altitude = heliState->setpoint.altitude;
			}

			heliState->mode.rollAxis = heliState->setpoint.rollAxis;
			if (heliState->mode.rollAxis == SB_CTRL_POS) {
				heliState->control.roll = heliState->setpoint.roll + oaroll;
			} else if (oaroll) {
				heliState->mode.rollAxis = SB_CTRL_POS;
				heliState->control.roll = oaroll;
			} else {
				heliState->control.roll = heliState->setpoint.roll;
			}

			heliState->mode.pitchAxis = heliState->setpoint.pitchAxis;
			if (heliState->mode.pitchAxis == SB_CTRL_POS) {
				heliState->control.pitch = heliState->setpoint.pitch + oapitch;
			} else if (oapitch) {
				heliState->mode.pitchAxis = SB_CTRL_POS;
				heliState->control.pitch = oapitch;
			} else {
				heliState->control.pitch = heliState->setpoint.pitch;
			}
			break;
		default:
			// not possible
			break;
	}
}

void sbIncrementTime(SBCommLoopSpec *comm, SBHeliStateRaw *heliState, unsigned int n_ms) {
	comm->timecount += n_ms;
	comm->timeinmode += n_ms;
	// The increment below are done only when the watchdog has not been
	// triggered, to avoid counter wrapping
	// printf("WD: %d\n",comm->cmd_watchdog);
	if ((comm->timeout_ctrl != 0xFFFF) && (comm->ctrl_watchdog < comm->timeout_ctrl)) {
		comm->ctrl_watchdog += n_ms;
	}
	if ((comm->timeout_cmd != 0xFFFF) && (comm->cmd_watchdog < comm->timeout_cmd)) {
		comm->cmd_watchdog += n_ms;
	}
}


