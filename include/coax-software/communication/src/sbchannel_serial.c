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
#include "com/sbchannel_serial.h"

#include <sys/types.h>
#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>


struct SBSerialChannelData {
	char * device;
	unsigned int speed;
	int rtscts;
	int fd;
	//structure definie dans /bits/termios.h
	struct termios newtio;
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

static int sbser_flush(void * _data);

struct SBSerialChannelData *sbSerialAlloc(const char *dev, unsigned int speed, int rtscts)
{
	struct SBSerialChannelData *data = NULL;
	data = (struct SBSerialChannelData*)malloc(sizeof(struct SBSerialChannelData));
	if (!data) return NULL;

	data->device = strdup(dev);
	data->speed = speed;
	data->rtscts = rtscts;
	data->fd = -1;

	return data;
}


static
int sbser_open(void * _data) 
{
	struct SBSerialChannelData *data = (struct SBSerialChannelData*)_data;

	printf("SBSerialChannelData: sbser_open\n");
#if 0
	data->fd = open(data->device, O_RDWR | O_NOCTTY | O_NDELAY); 
	if (data->fd < 0) {
		data->fd = -1;
		perror("sbser_open: open:");
		return -1;
	}
	cfmakeraw (&data->newtio);
	bzero(&data->newtio,sizeof(struct termios));
	cfsetospeed(&data->newtio,(data->speed));
	cfsetispeed(&data->newtio,(data->speed));
	data->newtio.c_iflag = 0;
	data->newtio.c_iflag = IGNBRK | IGNPAR | INPCK;
	data->newtio.c_oflag = 0;
	data->newtio.c_cflag = 0;
	data->newtio.c_cflag = CS8 | CSTOPB | CREAD | CLOCAL | PARODD;
	cfsetospeed(&data->newtio,(data->speed));
	cfsetispeed(&data->newtio,(data->speed));
	
	tcflush(data->fd, TCIFLUSH);
	tcsetattr(data->fd,TCSANOW,&data->newtio);
#else
	int sercmd = 0;
	char command[1024];
	unsigned int realspeed = data->speed;
	switch (data->speed) {
		case B9600: realspeed = 9600; break;
		case B38400: realspeed = 38400; break;
		case B57600: realspeed = 57600; break;
		case B115200: realspeed = 115200; break;
		default:break;
	}
	sprintf(command,"stty %scrtscts -echo -icanon raw %d -F %s",
			((data->rtscts)?"":"-"), realspeed, data->device);
	system(command);
	data->fd = open(data->device, O_RDWR | O_NOCTTY | O_NDELAY); 
	if (data->fd < 0) {
		data->fd = -1;
		perror("sbser_open: open:");
		return -1;
	}
	sercmd = TIOCM_RTS;
	if (data->rtscts) {
		ioctl(data->fd, TIOCMBIS, &sercmd); // set the RTS pin.
	}
#endif
	sbser_flush(_data);
	return 0;
}

static
int sbser_close(void * _data) 
{
	struct SBSerialChannelData *data = (struct SBSerialChannelData*)_data;
	if (data->fd>=0)
		close(data->fd);
	data->fd = -1;

	return 0;
}

static
int sbser_destroy(void * _data) 
{
	struct SBSerialChannelData *data = (struct SBSerialChannelData*)_data;

	if (data) {
		free(data->device);
		if (data->fd != -1) {
			sbser_close(_data);
		}
		free(data);
	}

	return 0;
}

static
int sbser_send(void * _data,const unsigned char *msg, size_t size) 
{
	struct SBSerialChannelData *data = (struct SBSerialChannelData*)_data;

	int wes;
	if (data->rtscts) {
        int sercmd = TIOCM_RTS;
		ioctl(data->fd, TIOCMBIS, &sercmd); // set the RTS pin.
        sercmd = TIOCM_DTR;
		ioctl(data->fd, TIOCMBIS, &sercmd); // set the RTS pin.
	}
	if (data->fd<0)
		return -1;
	wes=write(data->fd,msg,size);
	return wes;
}

static
int sbser_receive(void * _data, unsigned char *msg, size_t size) 
{
	struct SBSerialChannelData *data = (struct SBSerialChannelData*)_data;
	int w;

	if (data->fd<0)
		return 0;
	if (data->rtscts) {
        int sercmd = TIOCM_RTS;
		ioctl(data->fd, TIOCMBIS, &sercmd); // set the RTS pin.
        sercmd = TIOCM_DTR;
		ioctl(data->fd, TIOCMBIS, &sercmd); // set the RTS pin.
	}
	w = read(data->fd,msg,size);
	// if (w < 0) perror("sbser_receive:");
	return w;
}

static
int sbser_sendall(void * _data, const unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	static int nuser = 0;
	unsigned int sentbytes;
	double t1ms,t2ms,dtms,maxms;
	struct timeval tv1,tv2;
	nuser += 1;
	gettimeofday(&tv1,NULL);
	maxms = timeout_ms * 1e-3;
	t1ms = tv1.tv_sec+(tv1.tv_usec*1e-6);
	sentbytes = 0;
	while (sentbytes < size)
	{
		assert(nuser == 1);
		int r = sbser_send (_data, msg+sentbytes, size-sentbytes);
#if 0
		unsigned int i;
		printf("Sent %d: ",sentbytes+r);
		for (i=0;i<r;i++) {
			printf("%02X ",*(msg+sentbytes+i));
		}
		printf("\n");
#endif
		if (r > 0) sentbytes += r;
		gettimeofday(&tv2,NULL);
		t2ms = tv2.tv_sec+(tv2.tv_usec*1e-6);
		dtms = t2ms - t1ms;
#if 0
		printf("%f %f dtms : %f/%f\n",t1ms,t2ms,dtms,maxms);
#endif
		
		if ((timeout_ms>0) && (dtms >= maxms)) break;
	}
#if 0
	printf(" Finally sent %d bytes\n",sentbytes);
#endif
	nuser -= 1;
	return (sentbytes == size)?0:-1;
}

static
int sbser_waitbuffer(void * _data,  unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	static int nuser = 0;
	struct SBSerialChannelData *data = (struct SBSerialChannelData*)_data;
	fd_set rfs;
	// unsigned int i;
	unsigned int readbytes;
	double t1ms,t2ms,dtms,maxms;
	struct timeval tv1,tv2;
	struct timeval to = {0,0};
	maxms = timeout_ms*1e-3;
	gettimeofday(&tv1,NULL);
	readbytes = 0;
	nuser += 1;
	t1ms = tv1.tv_sec+tv1.tv_usec*1e-6;
#if 0
	printf("Waiting buffer\n");
#endif
	while (readbytes < size)
	{
		assert(nuser == 1);
		to.tv_sec = (timeout_ms/1000);
		to.tv_usec = (timeout_ms%1000) * 1000;
		FD_ZERO(&rfs);FD_SET(data->fd,&rfs);
		int r = select(data->fd+1,&rfs,NULL,NULL,&to);
		if (r < 1) {nuser-=1; return -1;}
		// at this point, at least one file is said to contains some data
		// so sbser_receive cannot returns 0
		r = sbser_receive(_data, msg+readbytes, size-readbytes );
		if (r == 0) {
			// the channel is corrupted;
			printf("The communication channel may be damaged (receive returns zero character after select)\n");
			nuser -= 1;
			return -2;
		}
#if 0
		{
			int i;
			printf("Received %d: ",readbytes+r);
			for (i=0;i<r;i++) {
				printf("%02X ",*(msg+readbytes+i));
			}
			printf("\n");
		}
#endif
		if (r>0) readbytes += r;

		gettimeofday(&tv2,NULL);
		t2ms = tv2.tv_sec+tv2.tv_usec*1e-6;
		dtms = t2ms - t1ms;
		if ((timeout_ms>0) && (dtms >= maxms)) break;
		dtms = maxms - dtms;
#if 0
		printf("%f %f dtms : %f/%f\n",t1ms,t2ms,dtms,maxms);
#endif
		to.tv_sec = (unsigned int)(dtms);
		to.tv_usec = (unsigned int)((dtms - to.tv_sec)*1e6);
	}
#if 0
	printf(" Finally received %d bytes\n",readbytes);
#endif
	nuser -= 1;
	return (readbytes == size)?0:-1;
}

static
int sbser_waitdata(void * _data, unsigned int timeout_ms) 
{
	fd_set rfs;
	struct SBSerialChannelData *data = (struct SBSerialChannelData*)_data;
	struct timeval to = {0,0};
	to.tv_sec = (timeout_ms/1000);
	to.tv_usec = (timeout_ms%1000) * 1000;
	FD_ZERO(&rfs);FD_SET(data->fd,&rfs);
	if (data->rtscts) {
        int sercmd = TIOCM_RTS;
		ioctl(data->fd, TIOCMBIS, &sercmd); // set the RTS pin.
        sercmd = TIOCM_DTR;
		ioctl(data->fd, TIOCMBIS, &sercmd); // set the RTS pin.
	}
	int r = select(data->fd+1,&rfs,NULL,NULL,(timeout_ms>0)?(&to):NULL);
	return (r>=1)?0:-1;
}

static
int sbser_flush(void * _data) 
{
#if 0
	struct SBSerialChannelData *data = (struct SBSerialChannelData*)_data;
	if (data->fd<0)
		return -1;
	tcdrain(data->fd);
	tcflush(data->fd, TCIFLUSH);
#else
	unsigned char dump=0xAE;
	while (sbser_receive(_data, &dump, 1 )>0) {
		// printf("flushing %02X\n",dump);
	}
#endif
	return 0;
}


static 
struct SBChannelFunction functions = {
	.destroy = sbser_destroy,
	.open = sbser_open,
	.close = sbser_close,

	.send = sbser_send,
	.receive = sbser_receive,

	.sendall = sbser_sendall,
	.waitbuffer = sbser_waitbuffer,
	.waitdata = sbser_waitdata,

	.flush = sbser_flush
};

struct SBChannelFunction* SB_SERIAL_FUNCTIONS = &functions;


