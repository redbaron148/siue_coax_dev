/*********************************************************************
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
*********************************************************************/

#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <time.h>

#include <com/sbchannel.h>
#include <com/sbmessage.h>
#include <com/sbapi.h>

//#define DEBUG(text) fprintf(stderr,"%s",text)
#define DEBUG(text) 
#define ERROR(text...) fprintf(stderr,text)

#define RECEIVE_TIMEOUT 500
#define SEND_TIMEOUT 500

int end = 0;
void sighdl(int n) {
	end ++;
	if (end >= 3) {
		kill(getpid(),SIGKILL);
	}
}

double now() {
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec + 1e-6 * tv.tv_usec;
}

int main(int argc, const char * argv[])
{
	char hostname[1024];
	unsigned int port;
	SBChannel channel[2];
	signal(SIGINT,sighdl);

	// TODO: add a good command line management, and create the channel as
	if (argc < 3) {
		fprintf(stderr,"Usage: %s <channel 0> <channel 1>\n",argv[0]);
		return -1;
	}

	switch (argv[1][0]) {
		case '/':
			sbChannelCreateSerial(channel+0,argv[1],115200,0); // real port, no flow control
			printf("Channel 0: Serial port %s\n",argv[1]);
			break;
		case '-':
			port = 5123;
			sscanf(argv[1]+1,"%d",&port);
			sbChannelCreateSocketUDP(channel+0,NULL,port); // simulator here
			printf("Channel 0: UDP Server port %d\n",port);
			break;
		default:
			port = 5123; hostname[0]=0;
			sscanf(argv[1],"%[^:]:%d",hostname,&port);
			sbChannelCreateSocketUDP(channel+0,hostname,5123); // simulator here
			printf("Channel 0: UDP Client %s port %d\n",hostname,port);
			break;
	}

	switch (argv[2][0]) {
		case '/':
			sbChannelCreateSerial(channel+1,argv[2],115200,0); // real port, no flow control
			printf("Channel 1: Serial port %s\n",argv[2]);
			break;
		case '-':
			port = 5123;
			sscanf(argv[2]+1,"%d",&port);
			sbChannelCreateSocketUDP(channel+1,NULL,port); // simulator here
			printf("Channel 1: UDP Server port %d\n",port);
			break;
		default:
			port = 5123; hostname[0]=0;
			sscanf(argv[2],"%[^:]:%d",hostname,&port);
			sbChannelCreateSocketUDP(channel+1,hostname,5123); // simulator here
			printf("Channel 1: UDP Client %s port %d\n",hostname,port);
			break;
	}

	sbChannelOpen(channel+0);
	sbChannelOpen(channel+1);

	printf("Channels opened. Entering the main loop\n");
	double lastprinttime = -100;
	unsigned int msgcount[2] = {0,0};
	while (!end) {
		SBSerialisedMessage sm;
		if (!sbChannelWaitData(channel+0,1)) {
			DEBUG("There is data on 0\n");
			int res = sbWaitRawMessage(channel+0,-1, &sm, RECEIVE_TIMEOUT);
			if (res) {
				// this should not fail, but just in case, discard anything 
				// in the message queue
				sbChannelFlush(channel+0);
				ERROR("Failed to read message on 0: %d\n",res);
			} else {
				res = sbSendMessage(channel+1, &sm, SEND_TIMEOUT);
				if (res) {
					sbChannelFlush(channel+1);
					ERROR("Failed to send message on 1: %d\n",res);
				}
				msgcount[0] += 1;
			}
		}

		if (!sbChannelWaitData(channel+1,1)) {
			DEBUG("There is data on 1\n");
			int res = sbWaitRawMessage(channel+1,-1, &sm, RECEIVE_TIMEOUT);
			if (res) {
				// this should not fail, but just in case, discard anything 
				// in the message queue
				sbChannelFlush(channel+1);
				ERROR("Failed to read message on 1: %d\n",res);
			} else {
				res = sbSendMessage(channel+0, &sm, SEND_TIMEOUT);
				if (res) {
					sbChannelFlush(channel+0);
					ERROR("Failed to send message on 0: %d\n",res);
				}
				msgcount[1] += 1;
			}
		}

		double t = now();
		if (t - lastprinttime > 5) {
			printf("Message received %d %.2f/s, %d %.2f/s\n",
					msgcount[0],msgcount[0]/(t-lastprinttime),
					msgcount[1],msgcount[1]/(t-lastprinttime));
			lastprinttime = t;
			msgcount[0] = msgcount[1] = 0;
		}
	}

	sbChannelDestroy(channel+0);
	sbChannelDestroy(channel+1);

	printf("Channels destroyed, exiting\n");


	return 0;
}
