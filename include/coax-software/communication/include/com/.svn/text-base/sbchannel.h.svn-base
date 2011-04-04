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
#ifndef SB_CHANNEL_H
#define SB_CHANNEL_H

#ifdef __cplusplus
extern "C" {
#endif


#ifdef PIC30
#ifndef NULL
typedef unsigned int size_t;
#define NULL ((void*)0)
#endif

#else
#include <stdlib.h>
#endif

#ifdef WIN32
#include <Windows.h>
#include <Winbase.h>
#endif

#ifdef SBC_HAS_PTHREAD
#include <pthread.h>
#endif

typedef enum {
	SB_CHANNEL_UDP,
	SB_CHANNEL_TCP,
	SB_CHANNEL_TCPFD,
	SB_CHANNEL_SERIAL,
	SB_CHANNEL_BTWIN,
	SB_CHANNEL_WINSOCK,
	SB_CHANNEL_UART_PIC30,
	SB_CHANNEL_COAX_UART1_PIC30,
	SB_CHANNEL_COAX_UART2_PIC30,
	SB_CHANNEL_COAX_BT_PIC30
} SBChannelType;

struct SBChannelFunction;

typedef struct {
	SBChannelType type;
#ifdef SBC_HAS_PTHREAD
	pthread_mutex_t mutex;
#endif
#ifdef WIN32
	CRITICAL_SECTION criticalSection;
#endif
	void * data;

	struct SBChannelFunction *functions;
} SBChannel;

#ifdef PIC30
#ifdef EPUCK
int sbChannelCreateUART1(SBChannel *channel);
#endif

#ifdef COAX
int sbChannelCreateCoaxUART1(SBChannel *channel);
int sbChannelCreateCoaxUART2(SBChannel *channel);
// Not yet implemented
// int sbChannelCreateCoaxBT(SBChannel *channel);
#endif
#endif

#ifdef WIN32
int sbChannelCreateBTWin(SBChannel *channel,const char *port, unsigned int speed);
int sbChannelCreateWinsockUDP(SBChannel *channel,const char *host,unsigned int port);
#endif

#ifdef LINUX
int sbChannelCreateSerial(SBChannel *channel,const char *dev,unsigned int speed,int rtscts);
int sbChannelCreateSocketUDP(SBChannel *channel,const char *host,unsigned int port);
int sbChannelCreateSocketTCP(SBChannel *channel,const char *host,unsigned int port);
int sbChannelCreateSocketFD(SBChannel *channel,int fd);
#endif
int sbChannelDestroy(SBChannel *channel);

int sbChannelOpen(SBChannel *channel);
int sbChannelClose(SBChannel *channel);

#ifndef PIC30
/*   These functions can be used to protect the channel when SBC_HAS_PTHREAD  */
/*   is set. They will fail returning -1 otherwise.  */
int sbChannelLock(SBChannel *channel);
int sbChannelUnlock(SBChannel *channel);
#endif

int sbChannelSend(SBChannel *channel, const unsigned char *data, size_t size);
int sbChannelReceive(SBChannel *channel, unsigned char *data, size_t size);

int sbChannelSendAll(SBChannel *channel, const unsigned char *data, size_t size,unsigned int timeout_ms);
int sbChannelWaitBuffer(SBChannel *channel, unsigned char *data, size_t size,unsigned int timeout_ms);
int sbChannelWaitData(SBChannel *channel, unsigned int timeout_ms);

int sbChannelFlush(SBChannel *channel);

int sbChannelSendString(SBChannel *channel,const char * string, unsigned int timeout_ms);


#ifdef __cplusplus
}
#endif

#endif /*   SB_CHANNEL_H  */
