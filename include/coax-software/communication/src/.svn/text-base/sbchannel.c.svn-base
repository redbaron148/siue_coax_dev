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
#include "com/sbchannel.h"
#include "com/sbchannel_fun.h"
#ifdef PIC30
#include "com/sbchannel_uart1.h"
#endif

#ifdef COAX
#include "com/sbchannel_coax_uart.h"
#endif

#ifdef LINUX
#include "com/sbchannel_serial.h"
#include "com/sbchannel_socket.h"
#endif

#ifdef WIN32
#include "com/sbchannel_btwin.h"
#include "com/sbchannel_winsock.h"
#endif

#ifdef PIC30
#ifdef EPUCK
int sbChannelCreateUART1(SBChannel *channel)
{
	channel->type = SB_CHANNEL_UART_PIC30;
	channel->functions = SB_UART_PIC30_FUNCTIONS;
	channel->data = NULL;
	sbUARTInit();
#ifdef SBC_HAS_PTHREAD
	pthread_mutex_init(&channel->mutex,NULL);
#endif
	return 0;
}
#endif

#ifdef COAX
int sbChannelCreateCoaxUART1(SBChannel *channel)
{
	channel->type = SB_CHANNEL_COAX_UART1_PIC30;
	channel->functions = SB_COAX_UART1_PIC30_FUNCTIONS;
	channel->data = NULL;
	sbCoaxUARTInit();
#ifdef SBC_HAS_PTHREAD
	pthread_mutex_init(&channel->mutex,NULL);
#endif
	return 0;
}

int sbChannelCreateCoaxUART2(SBChannel *channel)
{
	channel->type = SB_CHANNEL_COAX_UART2_PIC30;
	channel->functions = SB_COAX_UART2_PIC30_FUNCTIONS;
	channel->data = NULL;
	sbCoaxUARTInit();
#ifdef SBC_HAS_PTHREAD
	pthread_mutex_init(&channel->mutex,NULL);
#endif
	return 0;
}
#if 0 // not yet implemented
int sbChannelCreateCoaxBT(SBChannel *channel)
{
	channel->type = SB_CHANNEL_COAX_BT_PIC30;
	channel->functions = SB_COAX_BT_PIC30_FUNCTIONS;
	channel->data = NULL;
	sbCoaxBTInit();
#ifdef SBC_HAS_PTHREAD
	pthread_mutex_init(&channel->mutex,NULL);
#endif
	return 0;
}
#endif
#endif
#endif

#ifdef WIN32
int sbChannelCreateBTWin(SBChannel *channel,const char *port, unsigned int speed)
{
	channel->type = SB_CHANNEL_BTWIN;
	channel->functions = SB_BTWIN_FUNCTIONS;
	channel->data = sbBTWinAlloc(port,speed);
	InitializeCriticalSection(&channel->criticalSection);
	return (channel->data == NULL)?-1:0;
}

int sbChannelCreateWinsockUDP(SBChannel *channel,const char *host,unsigned int port)
{
	channel->type = SB_CHANNEL_WINSOCK;
	channel->functions = SB_WINSOCK_UDP_FUNCTIONS;
	channel->data = sbWinsockUDPAlloc(host,port);
	InitializeCriticalSection(&channel->criticalSection);
	return (channel->data == NULL)?-1:0;
}
#endif


#ifdef LINUX
#ifndef MACOSX
int sbChannelCreateSerial(SBChannel *channel,const char *dev,unsigned int speed, int rtscts)
{
	channel->type = SB_CHANNEL_SERIAL;
	channel->functions = SB_SERIAL_FUNCTIONS;
	channel->data = sbSerialAlloc(dev,speed,rtscts);
#ifdef SBC_HAS_PTHREAD
	pthread_mutex_init(&channel->mutex,NULL);
#endif
	return (channel->data == NULL)?-1:0;
}
#endif

int sbChannelCreateSocketUDP(SBChannel *channel,const char *host,unsigned int port)
{
	channel->type = SB_CHANNEL_UDP;
	channel->functions = SB_SOCKET_UDP_FUNCTIONS;
	channel->data = sbSocketUDPAlloc(host,port);
#ifdef SBC_HAS_PTHREAD
	pthread_mutex_init(&channel->mutex,NULL);
#endif
	return (channel->data == NULL)?-1:0;
}

int sbChannelCreateSocketTCP(SBChannel *channel,const char *host,unsigned int port)
{
	channel->type = SB_CHANNEL_TCP;
	channel->functions = SB_SOCKET_TCP_FUNCTIONS;
	channel->data = sbSocketTCPAlloc(host,port);
#ifdef SBC_HAS_PTHREAD
	pthread_mutex_init(&channel->mutex,NULL);
#endif
	return (channel->data == NULL)?-1:0;
}

int sbChannelCreateSocketFD(SBChannel *channel,int fd)
{
	channel->type = SB_CHANNEL_TCPFD;
	channel->functions = SB_SOCKET_FD_FUNCTIONS;
	channel->data = sbSocketFDAlloc(fd);
#ifdef SBC_HAS_PTHREAD
	pthread_mutex_init(&channel->mutex,NULL);
#endif
	return (channel->data == NULL)?-1:0;
}
#endif


int sbChannelDestroy(SBChannel *channel)
{
	int r = 0;
	if (channel->functions) {
#ifdef SBC_HAS_PTHREAD
		pthread_mutex_destroy(&channel->mutex);
#endif
#ifdef WIN32
		DeleteCriticalSection(&channel->criticalSection);
#endif
		r = channel->functions->destroy(channel->data);
		channel->data = NULL;
		channel->functions = NULL;
	}
	return r;
}

#ifndef PIC30

int sbChannelLock(SBChannel *channel)
{
#ifdef SBC_HAS_PTHREAD
	// printf("LOCKING\n");
	return pthread_mutex_lock(&channel->mutex);
#else
#ifdef WIN32
	EnterCriticalSection(&channel->criticalSection);
	return 0;
#else
	return -1;
#endif
#endif
}

int sbChannelUnlock(SBChannel *channel)
{
#ifdef SBC_HAS_PTHREAD
	// printf("UNLOCKING\n");
	return pthread_mutex_unlock(&channel->mutex);
#else
#ifdef WIN32
	LeaveCriticalSection(&channel->criticalSection);
	return 0;
#else
	return -1;
#endif
#endif
}

#endif

int sbChannelOpen(SBChannel *channel)
{
	return channel->functions->open(channel->data);
}

int sbChannelClose(SBChannel *channel)
{
	return channel->functions->close(channel->data);
}


int sbChannelSend(SBChannel *channel, const unsigned char *data, size_t size)
{
	return channel->functions->send(channel->data,data,size);
}

int sbChannelReceive(SBChannel *channel, unsigned char *data, size_t size)
{
	return channel->functions->receive(channel->data,data,size);
}


int sbChannelSendAll(SBChannel *channel, const unsigned char *data, size_t size,unsigned int timeout_ms)
{
	return channel->functions->sendall(channel->data,data,size,timeout_ms);
}

int sbChannelWaitBuffer(SBChannel *channel, unsigned char *data, size_t size,unsigned int timeout_ms)
{
	return channel->functions->waitbuffer(channel->data,data,size,timeout_ms);
}

int sbChannelWaitData(SBChannel *channel, unsigned int timeout_ms)
{
	return channel->functions->waitdata(channel->data,timeout_ms);
}


int sbChannelFlush(SBChannel *channel)
{
	return channel->functions->flush(channel->data);
}

int sbChannelSendString(SBChannel *channel,const char * string, 
		unsigned int timeout_ms)
{
	return sbChannelSendAll(channel,(const unsigned char*)string,strlen(string),timeout_ms);
}




