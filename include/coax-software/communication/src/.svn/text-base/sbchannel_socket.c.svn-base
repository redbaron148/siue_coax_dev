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
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/types.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <fcntl.h>
#include <stdio.h>
#include <assert.h>

#include "com/sbchannel_socket.h"

// Uncomment this to test the resilience of the API to 
// rotten socket (data corruption, packet loss, incomplete transmission)
// #define ROTTEN_SOCKET

#ifdef MACOSX
#define socklen_t unsigned int
#define MSG_NOSIGNAL SO_NOSIGPIPE
#endif

// #define DEBUG_SOCKET
#ifdef DEBUG_SOCKET
#define VPRINTF(X...) printf(X)
#else
#define VPRINTF(X...) 
#endif

#define UDP_PACKET_SIZE 8192

struct SBSocketChannelData {
	enum {SOCKET_UDP,SOCKET_TCP} type;
	int outputErrors;
	int m_sock;
	struct sockaddr_in destAddr;
	socklen_t destLen;

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

struct SBSocketChannelData *sbSocketUDPAlloc(const char *host, unsigned int port)
{
	struct SBSocketChannelData *data = NULL;
	data = (struct SBSocketChannelData*)malloc(sizeof(struct SBSocketChannelData));
	if (!data) return NULL;

	data->type = SOCKET_UDP;
	if (host) {
		data->host = strdup(host);
	} else {
		data->host = NULL;
	}
	data->port = port;
	data->m_sock = -1;
	data->outputErrors = 1;
	data->availData=0;
	data->readPtr=0;

	return data;
}


struct SBSocketChannelData *sbSocketTCPAlloc(const char *host, unsigned int port)
{
	struct SBSocketChannelData *data = NULL;
	data = (struct SBSocketChannelData*)malloc(sizeof(struct SBSocketChannelData));
	if (!data) return NULL;

	data->type = SOCKET_TCP;
	data->host = strdup(host);
	data->port = port;
	data->m_sock = -1;
	data->outputErrors = 1;
	data->availData=0;
	data->readPtr=0;

	return data;
}

struct SBSocketChannelData *sbSocketFDAlloc(int fd)
{
	struct SBSocketChannelData *data = NULL;
	data = (struct SBSocketChannelData*)malloc(sizeof(struct SBSocketChannelData));
	if (!data) return NULL;

	data->type = SOCKET_TCP;
	data->host = NULL;
	data->port = -1;
	data->m_sock = fd;
	data->outputErrors = 1;
	data->availData=0;
	data->readPtr=0;

	return data;
}

static
int sbsock_open_udp(void * _data) 
{
	struct SBSocketChannelData *that = (struct SBSocketChannelData*)_data;

	// Create
	that->m_sock = socket ( AF_INET, SOCK_DGRAM, IPPROTO_UDP );

#ifdef ROTTEN_SOCKET
	srand48(time(NULL));
#endif


	if (that->m_sock < 0) {
		return -1;
	}

	if (that->host) {
		that->destAddr.sin_family = AF_INET;
		that->destAddr.sin_port = htons ( that->port );
		that->destLen = sizeof(that->destAddr);

		int status = inet_pton ( AF_INET, that->host, &that->destAddr.sin_addr );
		if ( errno == EAFNOSUPPORT ) return -1;
		if (status <= 0)
		{
			struct hostent *_host_;
			_host_ =  gethostbyname(that->host);
			if (_host_ == NULL) return -1;
			memcpy(&that->destAddr.sin_addr.s_addr, _host_->h_addr, _host_->h_length);
		}
		if (connect ( that->m_sock, 
					(struct sockaddr * ) &that->destAddr, sizeof ( that->destAddr ) ) < 0) {
			if (that->outputErrors) perror("Connect");
			return -1;
		}
	} else {
		that->destLen = sizeof(that->destAddr);
		that->destAddr.sin_family = AF_INET;
		that->destAddr.sin_port = htons ( that->port );
		that->destAddr.sin_addr.s_addr = htonl ( INADDR_ANY );
		if (bind(that->m_sock,(struct sockaddr*)&that->destAddr,that->destLen)<0) {
			if (that->outputErrors) perror("Bind");
			return -1;
		}
	}


	int opts = fcntl ( that->m_sock, F_GETFL );
	if (opts >= 0)
		fcntl(that->m_sock,F_SETFL,opts | O_SYNC);

	// Set non blocking
	opts = fcntl ( that->m_sock, F_GETFL );
	if ( opts < 0 ) {
		return -1;
	}

	opts = ( opts | O_NONBLOCK );
	fcntl ( that->m_sock, F_SETFL,opts );

	return 0;
}


static
int sbsock_open_tcp(void * _data) 
{
	struct SBSocketChannelData *that = (struct SBSocketChannelData*)_data;

	// Create
	that->m_sock = socket ( AF_INET, SOCK_STREAM, 0 );

	if (that->m_sock < 0) {
		return -1;
	}

	int on = 1;
	if ( setsockopt ( that->m_sock, SOL_SOCKET, 
				SO_REUSEADDR, (  char* ) &on, sizeof ( on ) ) == -1 )
	{
		if (that->outputErrors) perror("Create");
		return -1;
	}
	if ( setsockopt ( that->m_sock, SOL_SOCKET, 
				SO_KEEPALIVE, (  char* ) &on, sizeof ( on ) ) == -1 )
	{
		if (that->outputErrors) perror("Create");
		return -1;
	}

	// Connect
	that->destAddr.sin_family = AF_INET;
	that->destAddr.sin_port = htons ( that->port );

	int status = inet_pton ( AF_INET, that->host, &that->destAddr.sin_addr );
	if ( errno == EAFNOSUPPORT ) return -1;
	if (status <= 0)
	{
		struct hostent *_host_;
		_host_ =  gethostbyname(that->host);
		if (_host_ == NULL) return -1;
		memcpy(&that->destAddr.sin_addr.s_addr, _host_->h_addr, _host_->h_length);
	}


	if (connect ( that->m_sock, 
			(struct sockaddr * ) &that->destAddr, sizeof ( that->destAddr ) ) < 0) {
		if (that->outputErrors) perror("Connect");
		return -1;
	}

	int opts = fcntl ( that->m_sock, F_GETFL );
	if (opts >= 0)
		fcntl(that->m_sock,F_SETFL,opts | O_SYNC);

	// Set non blocking
	opts = fcntl ( that->m_sock, F_GETFL );
	if ( opts < 0 ) {
		return -1;
	}

	opts = ( opts | O_NONBLOCK );
	fcntl ( that->m_sock, F_SETFL,opts );

	return 0;
}

static
int sbsock_close(void * _data) 
{
	struct SBSocketChannelData *data = (struct SBSocketChannelData*)_data;

	shutdown(data->m_sock,2);
	close(data->m_sock);
	return 0;
}

static
int sbsock_destroy(void * _data) 
{
	struct SBSocketChannelData *data = (struct SBSocketChannelData*)_data;
	if (data) {
		free(data->host);
		if (data->m_sock != -1) {
			sbsock_close(_data);
		}
		free(data);
	}

	return 0;
}

static
int sbsock_destroy_fd(void * _data) 
{
	struct SBSocketChannelData *data = (struct SBSocketChannelData*)_data;
	if (data) {
		free(data);
	}

	return 0;
}

static
int sbsock_send_tcp(void * _data,const unsigned char *msg, size_t size) 
{
	struct SBSocketChannelData *that = (struct SBSocketChannelData*)_data;
	int status = send ( that->m_sock, msg, size, MSG_NOSIGNAL );
	if ((status<0) && that->outputErrors) perror("sbsock_send_tcp");
	return status;
}

static
int sbsock_send_udp(void * _data,const unsigned char *msg, size_t size) 
{
	struct SBSocketChannelData *that = (struct SBSocketChannelData*)_data;

#ifdef ROTTEN_SOCKET
	double coin = drand48();
	if (coin < 0.05) {
		fprintf(stderr,"RottenSocket: ignored send\n");
		return 0;
	} else if (coin < 0.08) {
		fprintf(stderr,"RottenSocket: corrupted length\n");
		if (size > 3) size -= 3;
	} else if ((size>0) && (coin < 0.1)) {
		fprintf(stderr,"RottenSocket: corrupted message\n");
		assert(size <= 86); // SB_MAX_MESSAGE_SIZE
		size_t index = (size_t)(drand48()*size);
		if (index >= size) index = size-1;
		unsigned char newmsg[86];
		memcpy(newmsg,msg,size);
		newmsg[index] = (unsigned char)(drand48()*255);
		int status = sendto ( that->m_sock, newmsg, size, MSG_NOSIGNAL,
				(struct sockaddr*)&that->destAddr, that->destLen);
		return status;
	}
#endif

#ifdef MACOSX
	int status = send ( that->m_sock, msg, size, MSG_NOSIGNAL);
#else
	int status = sendto ( that->m_sock, msg, size, MSG_NOSIGNAL,
			(struct sockaddr*)&that->destAddr, that->destLen);
	VPRINTF("Sending to %d\n",ntohs(that->destAddr.sin_port));
#endif
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
	struct SBSocketChannelData *that = (struct SBSocketChannelData*)_data;
	VPRINTF("0 - Receiving %d bytes\n",size);
	// First try to use bufferised data
	if (that->availData) {
		unsigned int n = size;
		if (n > (that->availData-that->readPtr)) {
			n = (that->availData-that->readPtr);
		}
		memcpy(msg,that->buffer+that->readPtr,n);
		that->readPtr += n;
		VPRINTF("1 - Used %d bytes from buffer, %d avail\n",
				n, that->availData-that->readPtr);
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
	int status = recvfrom ( that->m_sock, that->buffer, UDP_PACKET_SIZE , 0 , 
			(struct sockaddr*)&that->destAddr, &that->destLen);
	VPRINTF("Receiving from %d\n",ntohs(that->destAddr.sin_port));

	if (status<0) {
		if (that->outputErrors) perror("sbsock_receive_udp");
	} else {
		// And use what is required from this packet, keeping the rest
		// for later
		unsigned int n = size;
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
		VPRINTF("3 - Used %d bytes from buffer, %d avail\n",
				n, that->availData-that->readPtr);
		if (that->readPtr >= that->availData) {
			that->readPtr = 0;
			that->availData = 0;
		}
		return n;
	}
	return status;
}

static
int sbsock_receive_tcp(void * _data, unsigned char *msg, size_t size) 
{
	struct SBSocketChannelData *that = (struct SBSocketChannelData*)_data;
	int status = recv ( that->m_sock, msg, size , 0 );
	if ((status<0) && that->outputErrors) perror("sbsock_receive_tcp");
	return status;
}

static
int sbsock_sendall(void * _data, const unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	struct SBSocketChannelData *that = (struct SBSocketChannelData*)_data;
	unsigned int sentbytes;
	double t1ms,t2ms,dtms,maxms;
	struct timeval tv1,tv2;
	gettimeofday(&tv1,NULL);
	maxms = timeout_ms * 1e-3;
	t1ms = tv1.tv_sec+(tv1.tv_usec*1e-6);
	sentbytes = 0;
	while (sentbytes < size)
	{
		int r;
		switch (that->type) {
			case SOCKET_UDP:
				r = sbsock_send_udp (_data, msg+sentbytes, size-sentbytes);
				break;
			case SOCKET_TCP:
				r = sbsock_send_tcp (_data, msg+sentbytes, size-sentbytes);
				break;
		}
		if (r < 0) {return r;}
		if (r > 0) sentbytes += r;
		//VPRINTF("Sent %d ",sentbytes);
		gettimeofday(&tv2,NULL);
		t2ms = tv2.tv_sec+(tv2.tv_usec*1e-6);
		dtms = t2ms - t1ms;
		//VPRINTF("%f %f dtms : %f/%f\n",t1ms,t2ms,dtms,maxms);
		
		if ((timeout_ms>0)&&(dtms >= maxms)) break;
	}
	//VPRINTF(" Finally sent %d bytes\n",sentbytes);
	return (sentbytes == size)?0:-1;
}

static
int sbsock_waitbuffer_tcp(void * _data,  unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	struct SBSocketChannelData *that = (struct SBSocketChannelData*)_data;
	fd_set rfs;
	unsigned int readbytes;
	double t1ms,t2ms,dtms,maxms;
	struct timeval tv1,tv2;
	struct timeval to = {0,0};
	maxms = timeout_ms*1e-3;
	to.tv_sec = (timeout_ms/1000);
	to.tv_usec = (timeout_ms%1000) * 1000;
	gettimeofday(&tv1,NULL);
	readbytes = 0;
	t1ms = tv1.tv_sec+tv1.tv_usec*1e-6;
	while (readbytes < size)
	{
		FD_ZERO(&rfs);FD_SET(that->m_sock,&rfs);
		int r = select(that->m_sock+1,&rfs,NULL,NULL,&to);
		if (r < 1) return -1;
		r = sbsock_receive_tcp (_data, msg+readbytes, size-readbytes );
		if (r>0) readbytes += r;
		if (r < 1) return -1;
		VPRINTF("Rec %d\n",readbytes);

		gettimeofday(&tv2,NULL);
		t2ms = tv2.tv_sec+tv2.tv_usec*1e-6;
		dtms = t2ms - t1ms;
		if ((timeout_ms>0) && (dtms >= maxms)) break;
		dtms = maxms - dtms;
		to.tv_sec = (unsigned int)(dtms);
		to.tv_usec = (unsigned int)((dtms - to.tv_sec)*1e6);
	}
	VPRINTF("Received %d bytes out %d\n",readbytes,size);
	return (readbytes == size)?0:-1;
}

static
int sbsock_waitbuffer_udp(void * _data,  unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	struct SBSocketChannelData *that = (struct SBSocketChannelData*)_data;
	fd_set rfs;
	unsigned int readbytes;
	double t1ms,t2ms,dtms,maxms;
	struct timeval tv1,tv2;
	struct timeval to = {0,0};
	maxms = timeout_ms*1e-3;
	to.tv_sec = (timeout_ms/1000);
	to.tv_usec = (timeout_ms%1000) * 1000;
	gettimeofday(&tv1,NULL);
	readbytes = 0;
	t1ms = tv1.tv_sec+tv1.tv_usec*1e-6;
	while (readbytes < size)
	{
		int r;
		if (!that->availData) {
			FD_ZERO(&rfs);FD_SET(that->m_sock,&rfs);
			r = select(that->m_sock+1,&rfs,NULL,NULL,&to);
			if (r < 1) return -1;
		}
		r = sbsock_receive_udp (_data, msg+readbytes, size-readbytes );
		if (r>0) readbytes += r;
		if (r < 1) return -1;
		VPRINTF("Rec %d\n",readbytes);

		gettimeofday(&tv2,NULL);
		t2ms = tv2.tv_sec+tv2.tv_usec*1e-6;
		dtms = t2ms - t1ms;
		if ((timeout_ms>0) && (dtms >= maxms)) break;
		dtms = maxms - dtms;
		to.tv_sec = (unsigned int)(dtms);
		to.tv_usec = (unsigned int)((dtms - to.tv_sec)*1e6);
	}
	VPRINTF("Received %d bytes out %d\n",readbytes,size);
	return (readbytes == size)?0:-1;
}

static
int sbsock_waitdata_tcp(void * _data, unsigned int timeout_ms) 
{
	struct SBSocketChannelData *that = (struct SBSocketChannelData*)_data;
	fd_set rfs;
	struct timeval to = {0,0};
	to.tv_sec = (timeout_ms/1000);
	to.tv_usec = (timeout_ms%1000) * 1000;
	FD_ZERO(&rfs);FD_SET(that->m_sock,&rfs);
	int r = select(that->m_sock+1,&rfs,NULL,NULL,(timeout_ms>0)?(&to):NULL);
	return (r >= 1)?0:-1;
}

static
int sbsock_waitdata_udp(void * _data, unsigned int timeout_ms) 
{
	struct SBSocketChannelData *that = (struct SBSocketChannelData*)_data;
	fd_set rfs;
	struct timeval to = {0,0};
	if (that->availData) {
		return 0;
	}

	to.tv_sec = (timeout_ms/1000);
	to.tv_usec = (timeout_ms%1000) * 1000;
	FD_ZERO(&rfs);FD_SET(that->m_sock,&rfs);
	int r = select(that->m_sock+1,&rfs,NULL,NULL,(timeout_ms>0)?(&to):NULL);
	return (r >= 1)?0:-1;
}

static
int sbsock_flush_tcp(void * _data) 
{
	unsigned char trash[32];
	while (sbsock_receive_tcp(_data,trash,32)>0);
	return 0;
}

static
int sbsock_flush_udp(void * _data) 
{
	struct SBSocketChannelData *that = (struct SBSocketChannelData*)_data;
	that->availData = 0;
	that->readPtr = 0;
	return 0;
}

static
int failure_func(void * _data)
{
	return -1;
}

static 
struct SBChannelFunction functions_tcp = {
	.destroy = sbsock_destroy,
	.open = sbsock_open_tcp,
	.close = sbsock_close,

	.send = sbsock_send_tcp,
	.receive = sbsock_receive_tcp,

	.sendall = sbsock_sendall,
	.waitbuffer = sbsock_waitbuffer_tcp,
	.waitdata = sbsock_waitdata_tcp,

	.flush = sbsock_flush_tcp
};

struct SBChannelFunction* SB_SOCKET_TCP_FUNCTIONS = &functions_tcp;

static 
struct SBChannelFunction functions_fd = {
	.destroy = sbsock_destroy_fd,
	.open = failure_func,
	.close = failure_func,

	.send = sbsock_send_tcp,
	.receive = sbsock_receive_tcp,

	.sendall = sbsock_sendall,
	.waitbuffer = sbsock_waitbuffer_tcp,
	.waitdata = sbsock_waitdata_tcp,

	.flush = sbsock_flush_tcp
};

struct SBChannelFunction* SB_SOCKET_FD_FUNCTIONS = &functions_fd;

static 
struct SBChannelFunction functions_udp = {
	.destroy = sbsock_destroy,
	.open = sbsock_open_udp,
	.close = sbsock_close,

	.send = sbsock_send_udp,
	.receive = sbsock_receive_udp,

	.sendall = sbsock_sendall,
	.waitbuffer = sbsock_waitbuffer_udp,
	.waitdata = sbsock_waitdata_udp,

	.flush = sbsock_flush_udp
};

struct SBChannelFunction* SB_SOCKET_UDP_FUNCTIONS = &functions_udp;


