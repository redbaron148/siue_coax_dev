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
#include <Winsock2.h>
#include <strsafe.h>
#include <windows.h>
#include <time.h>
#include <stdio.h>
#include <assert.h>
#include <stdio.h>
#include <windows.h>
#pragma comment(lib, "wininet.lib")



#include "com/sbchannel_socket.h"

// Uncomment this to test the resilience of the API to 
// rotten socket (data corruption, packet loss, incomplete transmission)
// #define ROTTEN_SOCKET

#define UDP_PACKET_SIZE 8192

struct SBWinsockChannelData {
	enum {SOCKET_UDP,SOCKET_TCP} type;
	int outputErrors;

	int connected;
	SOCKET sock;
	struct sockaddr_in destAddr;
	int destLen;

	unsigned char buffer[UDP_PACKET_SIZE];
	unsigned int availData;
	unsigned int readPtr;

	int port;
	char * host;
};

/*
	int (*destroy)(void*);

	int (*open)(void*);
	int (*close)(void*);

	int (*send)(void*,const unsigned char *data, size_t size);
	int (*receive)(void*,unsigned char *data, size_t size);

	int (*sendall)(void*,const unsigned char *data, size_t size,unsigned int timeout_ms);
	int (*waitbuffer)(void*,unsigned char *data, size_t size,unsigned int timeout_ms);
	int (*waitdata)(void*,unsigned int timeout_ms);

	int (*flush)(void*);
*/

struct SBWinsockChannelData *sbWinsockUDPAlloc(const char *host, unsigned int port)
{
	struct SBWinsockChannelData *data = NULL;
	data = (struct SBWinsockChannelData*)malloc(sizeof(struct SBWinsockChannelData));
	if (!data) return NULL;

	data->type = SOCKET_UDP;
	if (host) {
		data->host = _strdup(host);
	} else {
		data->host = NULL;
	}
	data->port = port;
	data->sock = INVALID_SOCKET;
	data->outputErrors = 1;
	data->availData=0;
	data->readPtr=0;
	data->connected=0;

	return data;
}


#ifndef __GNUC__
#define EPOCHFILETIME (116444736000000000i64)
#else
#define EPOCHFILETIME (116444736000000000LL)
#endif

static double now() {
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


static
int sbsock_open_udp(void * _data) 
{
	struct SBWinsockChannelData *that = (struct SBWinsockChannelData*)_data;
	ULONG 		ulRetCode = 0;
	WSADATA		WSAData = {0};

#ifdef ROTTEN_SOCKET
	srand48(time(NULL));
#endif

	that->connected = 0;

	ulRetCode = WSAStartup(MAKEWORD(2, 2), &WSAData);
	if ( 0 != ulRetCode ) { // "zero" per SDK
		if (that->outputErrors) {
			printf("-FATAL- | Unable to initialize Winsock version 2.2\n");
		}
		return -1;
	}

	// Create
	that->sock = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP );
	if (INVALID_SOCKET == that->sock) {
		if (that->outputErrors) {
			printf("=CRITICAL= | socket() call failed. WSAGetLastError = [%d]\n", WSAGetLastError());
		}
		return -1;
	}

	that->destAddr.sin_family = AF_INET;
	that->destLen = sizeof(that->destAddr);
	if (that->host != NULL) {
		/*----------------------
		// The sockaddr_in structure specifies the address family,
		// IP address, and port of the server to be connected to.
		*/
		that->destAddr.sin_addr.s_addr = inet_addr(that->host);
		that->destAddr.sin_port = htons(that->port);

		if ( SOCKET_ERROR == connect(that->sock,
			(struct sockaddr *) &that->destAddr,
			that->destLen) ) {
				if (that->outputErrors) {
					printf("=CRITICAL= | connect(%s,%d) call failed. WSAGetLastError=[%d]\n", that->host,that->port,WSAGetLastError());
				}
				closesocket(that->sock);
				that->sock = INVALID_SOCKET;
				return -1;
		}
		that->connected = 1;
	} else {
		/* Get host name of this computer */
		char host_name[256]="";
		struct hostent * hp;
		gethostname(host_name, sizeof(host_name));
		hp = gethostbyname(host_name);

		/* Check for NULL pointer */
		if (hp == NULL)
		{
			fprintf(stderr, "Could not get host name.\n");
			closesocket(that->sock);
			that->sock = INVALID_SOCKET;
			WSACleanup();
			return -1;
		}

		/* Assign the address */
		that->destAddr.sin_port = htons(that->port);
		that->destAddr.sin_addr.S_un.S_un_b.s_b1 = hp->h_addr_list[0][0];
		that->destAddr.sin_addr.S_un.S_un_b.s_b2 = hp->h_addr_list[0][1];
		that->destAddr.sin_addr.S_un.S_un_b.s_b3 = hp->h_addr_list[0][2];
		that->destAddr.sin_addr.S_un.S_un_b.s_b4 = hp->h_addr_list[0][3];
		if ( SOCKET_ERROR == bind(that->sock,
			(struct sockaddr *) &that->destAddr,
			that->destLen) ) {
				if (that->outputErrors) {
					printf("=CRITICAL= | bind(%d) call failed. WSAGetLastError=[%d]\n", that->port,WSAGetLastError());
				}
				closesocket(that->sock);
				that->sock = INVALID_SOCKET;
				return -1;
		}
	}

	{
		// Set non blocking
		u_long nNoBlock = 1;
		ioctlsocket(that->sock, FIONBIO, &nNoBlock);
	}
	return 0;
}

static
int sbsock_flush_udp(void * _data) 
{
	struct SBWinsockChannelData *that = (struct SBWinsockChannelData*)_data;
	that->availData = 0;
	that->readPtr = 0;
	return 0;
}


static
int sbsock_close(void * _data) 
{
	struct SBWinsockChannelData *data = (struct SBWinsockChannelData*)_data;
	sbsock_flush_udp(_data);

	if ( INVALID_SOCKET != data->sock ) {
		closesocket(data->sock);
		data->sock = INVALID_SOCKET;
	}
	data->connected = 0;
	return 0;
}

static
int sbsock_destroy(void * _data) 
{
	struct SBWinsockChannelData *data = (struct SBWinsockChannelData*)_data;
	if (data) {
		free(data->host);
		if (data->sock != INVALID_SOCKET) {
			sbsock_close(_data);
		}
		free(data);
	}

	return 0;
}



static
int sbsock_send_udp(void * _data,const unsigned char *msg, size_t size) 
{
	struct SBWinsockChannelData *that = (struct SBWinsockChannelData*)_data;
	int status;

	if (!that->connected) {
		return -1;
	}

#ifdef ROTTEN_SOCKET
	double coin = drand48();
	if (coin < 0.05) {
		fprintf(stderr,"RottenSocket: ignored send\n");
		return 0;
	} else if (coin < 0.08) {
		fprintf(stderr,"RottenSocket: corrupted length\n");
		if (size > 3) size -= 3;
	} else if ((size>0) && (coin < 0.1)) {
		int status;
		size_t index;
		unsigned char newmsg[86];
		fprintf(stderr,"RottenSocket: corrupted message\n");
		assert(size <= 86); // SB_MAX_MESSAGE_SIZE
		index = (size_t)(drand48()*size);
		if (index >= size) index = size-1;
		memcpy(newmsg,msg,size);
		newmsg[index] = (unsigned char)(drand48()*255);
		status = sendto ( that->sock, newmsg, size, 0,
				(struct sockaddr*)&that->destAddr, that->destLen);
		return status;
	}
#endif

#if 1
	// This does not work for some reason. Never mind, as long as the winsock
	// channel are only used for client of the simulator/helicopter. This would
	// have to be solved otherwise
	status = sendto ( that->sock, msg, size, 0,
			(struct sockaddr*)&that->destAddr, that->destLen);
#else
	status = send ( that->sock, msg, size, 0);
#endif
	if ((status == SOCKET_ERROR) && that->outputErrors) {
		printf("send() call failed w/socket = [0x%X], szData = [%p], dataLen = [%d]. WSAGetLastError=[%d]\n", 
				that->sock, msg, size, WSAGetLastError());
	}
#ifdef DEBUG_SOCKET
	{
		unsigned int i;
		printf("Sent %d bytes out of %d: ",status,size);
		for (i=0;i<status;i++) {
			printf("%02X ",msg[i]);
		}
		printf("\n");
	}
#endif
	if ((status<0) && that->outputErrors) perror("sbsock_send_udp");
	return status;
}

static
int sbsock_receive_udp(void * _data, unsigned char *msg, size_t size) 
{
	int status;
	struct SBWinsockChannelData *that = (struct SBWinsockChannelData*)_data;
	// First try to use bufferised data
	if (that->availData) {
		unsigned int n = size;
		if (n > (that->availData-that->readPtr)) {
			n = (that->availData-that->readPtr);
		}
		memcpy(msg,that->buffer+that->readPtr,n);
		that->readPtr += n;
		if (that->readPtr >= that->availData) {
			that->readPtr = 0;
			that->availData = 0;
		}
		size -= n;
		msg += n;
		if (size == 0) {
			return n;
		}
	}

	// If there is still something to read, then try to receive a packet
#if 1
	status = recvfrom ( that->sock, that->buffer, UDP_PACKET_SIZE , 0 , 
			(struct sockaddr*)&that->destAddr, &that->destLen);
#else
	status = recv ( that->sock, that->buffer, UDP_PACKET_SIZE , 0 );
#endif

	if ((status == SOCKET_ERROR) && that->outputErrors) {
		printf("recv() call failed. WSAGetLastError=[%d]\n", WSAGetLastError());
		if (WSAGetLastError() == WSAECONNRESET) {
			sbsock_close(that);
			Sleep(500);
			sbsock_open_udp(that);
		}
	} else {
		// And use what is required from this packet, keeping the rest
		// for later
		unsigned int n = size;
		that->connected = 1;
		that->availData = status;
		that->readPtr = 0;
#ifdef DEBUG_SOCKET
		{
			unsigned int i;
			printf("2 - Received %d new bytes: ",status);
			for (i=0;i<status;i++) {
				printf("%02X ",that->buffer[i]);
			}
			printf("\n");
		}
#endif
		if (n > (that->availData-that->readPtr)) {
			n = (that->availData-that->readPtr);
		}
		memcpy(msg,that->buffer+that->readPtr,n);
		that->readPtr += n;
		if (that->readPtr >= that->availData) {
			that->readPtr = 0;
			that->availData = 0;
		}
		return n;
	}
	return status;
}


static
int sbsock_sendall(void * _data, const unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	struct SBWinsockChannelData *that = (struct SBWinsockChannelData*)_data;
	unsigned int sentbytes;
	double t1ms,t2ms,dtms,maxms;
	maxms = timeout_ms * 1e-3;
	t1ms = now();
	sentbytes = 0;
	while (sentbytes < size)
	{
		int r;
		r = sbsock_send_udp (_data, msg+sentbytes, size-sentbytes);
		if (r < 0) {return r;}
		if (r > 0) sentbytes += r;
		t2ms = now();
		dtms = t2ms - t1ms;
		
		if ((timeout_ms>0)&&(dtms >= maxms)) break;
	}
	return (sentbytes == size)?0:-1;
}


static
int sbsock_waitbuffer_udp(void * _data,  unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	struct SBWinsockChannelData *that = (struct SBWinsockChannelData*)_data;
	fd_set rfs;
	unsigned int readbytes;
	double t1ms,t2ms,dtms,maxms;
	struct timeval to = {0,0};
	maxms = timeout_ms*1e-3;
	to.tv_sec = (timeout_ms/1000);
	to.tv_usec = (timeout_ms%1000) * 1000;
	readbytes = 0;
	t1ms = now();
	while (readbytes < size)
	{
		int r;
		if (!that->availData) {
			FD_ZERO(&rfs);FD_SET(that->sock,&rfs);
			r = select(that->sock+1,&rfs,NULL,NULL,&to);
			if (r < 1) return -1;
		}
		r = sbsock_receive_udp (_data, msg+readbytes, size-readbytes );
		if (r>0) readbytes += r;
		if (r < 1) return -1;

		t2ms = now();
		dtms = t2ms - t1ms;
		if ((timeout_ms>0) && (dtms >= maxms)) break;
		dtms = maxms - dtms;
		to.tv_sec = (unsigned int)(dtms);
		to.tv_usec = (unsigned int)((dtms - to.tv_sec)*1e6);
	}
	return (readbytes == size)?0:-1;
}


static
int sbsock_waitdata_udp(void * _data, unsigned int timeout_ms) 
{
	struct SBWinsockChannelData *that = (struct SBWinsockChannelData*)_data;
	fd_set rfs;
	struct timeval to = {0,0};
	int r;
	if (that->availData) {
		return 0;
	}

	to.tv_sec = (timeout_ms/1000);
	to.tv_usec = (timeout_ms%1000) * 1000;
	FD_ZERO(&rfs);FD_SET(that->sock,&rfs);
	r = select(that->sock+1,&rfs,NULL,NULL,(timeout_ms>0)?(&to):NULL);
	return (r >= 1)?0:-1;
}


static
int failure_func(void * _data)
{
	return -1;
}
// Visual studio does not support named initialization (C99 standard...)
static 
struct SBChannelFunction functions_udp = {
	sbsock_destroy,
	sbsock_open_udp,
	sbsock_close,

	sbsock_send_udp,
	sbsock_receive_udp,

	sbsock_sendall,
	sbsock_waitbuffer_udp,
	sbsock_waitdata_udp,

	sbsock_flush_udp
};

struct SBChannelFunction* SB_WINSOCK_UDP_FUNCTIONS = &functions_udp;


