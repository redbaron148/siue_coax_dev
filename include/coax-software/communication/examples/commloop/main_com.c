/*************************************************************
 *
 * Low-level controller with API server for the Skybotix AG
 * helicopters, and particularly COAX
 *
 * Developed by:
 *  * Cedric Pradalier: cedric.pradalier@skybotix.ch
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
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef LINUX
#include <pthread.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#endif

#ifdef _MSC_VER
#define inline __inline
#endif

#include <com/sbconst.h>
#include <com/sbchannel.h>
#include <com/sbcommloop.h>

#include "utils.h"
#include "state.h"

#ifdef LINUX
pthread_t comm_tid = -1;
#endif
#ifdef WIN32
HANDLE comm_tid;
#endif
SBCommLoopSpec *comm = &gcomm;

// Variable to manage message frequencies
unsigned int messagePeriod[2] = {0,0};
double lastMessage[2] = {-1,-1};
void setPeriodicState(unsigned int source, unsigned int period_ms)
{
	if(source < 2) {
		messagePeriod[source] = period_ms;
	}
}


void sb_iterate() {
	unsigned int i;
	sbIncrementTime(&gcomm,&heliState,10);
	// Update the global variable describing the state
	gcomm.updateHeliState();

	// Check if it is time to send a message on one of the channel.
	for (i=0;i<2;i++) {
		if ((messagePeriod[i]>0) && (now() - lastMessage[i] > messagePeriod[i]*1e-3)) {
			if (!gcomm.stateReady[i]) {
				// We only send if we finished sending the previous one
				if (sbStateEncode(&gcomm.serialisedState,0,&heliState)) {
					// ...
				} else {
					gcomm.stateReady[i] = 1;
				}
				lastMessage[i] = now();
			}
		}
	}

	// Update state machine
	sbStateMachine(&gcomm,&heliState);

	// Compute helicopter control
	if (heliState.mode.navigation == SB_NAV_RAW) {
		// ... = heliState.setpoint.motor1;
		// ... = heliState.setpoint.motor2;
		// ... = heliState.setpoint.servo1;
		// ... = heliState.setpoint.servo2;
	} else {
		// Reset the raw input, just in case
		heliState.setpoint.motor1 = heliState.setpoint.motor2 = 0;
		heliState.setpoint.servo1 = heliState.setpoint.servo2 = 0;

		switch (heliState.mode.rollAxis) {
			case SB_CTRL_MANUAL:
				// Not implemented, but should simulate the RC command here
				break;
			case SB_CTRL_POS:
				// rollstar = heliState.control.roll*M_PI/1800.;
				// do something with rollstar == roll servo position
				break;
			case SB_CTRL_VEL:
				// Not correct, but will be OK for now
				if (heliState.coaxspeed.state == 
						(COAXSPEED_AVAILABLE | COAXSPEED_VALID_MEASUREMENT)) {
					// rollstar = heliState.control.roll/1000.;
					// do something with rollstar == desired lateral speed
				} else {
					// not implemented
				}
				break;
			case SB_CTRL_NONE:
			case SB_CTRL_REL:
			default:
				// not implemented
				break;
		}
		switch (heliState.mode.pitchAxis) {
			case SB_CTRL_MANUAL:
				// Not implemented, but should simulate the RC command here
				break;
			case SB_CTRL_POS:
				// pitchstar = heliState.control.pitch*M_PI/1800.;
				// do something with pitchstar == pitch servo position
				break;
			case SB_CTRL_VEL:
				// Not correct, but will be OK for now
				if (heliState.coaxspeed.state == 
						(COAXSPEED_AVAILABLE | COAXSPEED_VALID_MEASUREMENT)) {
					// pitchstar = heliState.control.pitch/1000.;
					// do something with pitchstar == desired longitudianl speed
				} else {
					// not implemented
				}
				break;
			case SB_CTRL_NONE:
			case SB_CTRL_REL:
			default:
				// not implemented
				break;
		}


		switch (heliState.mode.yawAxis) {
			case SB_CTRL_MANUAL:
				// Not implemented, but should simulate the RC command here
				break;
			case SB_CTRL_VEL:
				// yawstar = heliState.control.yaw*M_PI/1800.;
				// do something with yawstar == yaw velocity
				break;
			case SB_CTRL_POS:
				// yawstar = heliState.control.yaw*M_PI/1800.;
				// do something with yawstar == yaw position
				break;
			case SB_CTRL_REL:
			case SB_CTRL_NONE:
			default:
				// not implemented
				break;
		}

		switch (heliState.mode.altAxis) {
			case SB_CTRL_MANUAL | SB_CTRL_POS:
			case SB_CTRL_MANUAL | SB_CTRL_REL:
			case SB_CTRL_MANUAL | SB_CTRL_VEL:
				// Simulate the RC mode in position or relative position
				break;
			case SB_CTRL_MANUAL | SB_CTRL_FORCE:
				// Simulate the RC mode in thrust mode
				break;
			case SB_CTRL_VEL:
				// zstar = heliState.control.altitude/1000.0;
				// do something with zstar = altitude velocity
				break;
			case SB_CTRL_POS:
				// zstar = heliState.control.altitude/1000.0;
				// do something with zstar = altitude absolute position
				break;
			case SB_CTRL_REL:
				// zstar = heliState.control.altitude/1000.0;
				// do something with zstar = relative distance to the closest
				// object
				break;
			case SB_CTRL_NONE:
			default:
				// not implemented
				break;
		}
	}
}

void simulation_loop()
{
	while (1) {
		printf(".");fflush(stdout);
		// Set the helicopter state, and prepare the control variables
		sb_iterate();
		// Run the real simulation based on the control
		//
		//
#ifdef LINUX
		usleep(10000);
#endif
#ifdef WIN32
		Sleep(10);
#endif
	}
}

#ifdef LINUX
// Thread running the communication loop
void *commthread(void* dummy)
{
	sbCommLoop(&gcomm,&heliState);
	return NULL;
}

void startCommThread() 
{
	pthread_create(&comm_tid,NULL,commthread,NULL);
}

void stopCommThread() 
{
	pthread_cancel(comm_tid);
}
#endif

#ifdef WIN32
DWORD WINAPI commthread(LPVOID _context) {
	sbCommLoop(&gcomm,&heliState);
	return 0;
}

void startCommThread() 
{
	DWORD tid;
	comm_tid = CreateThread(NULL,0,commthread,NULL,0,&tid);
}

void stopCommThread() 
{
	WaitForSingleObject(comm_tid,INFINITE);
	CloseHandle(comm_tid);
}
#endif

int main(int argc, char *argv[]) {

	// Will be called in sbStateMachine (from control function)
	sbCommLoopInit(&gcomm,0x100);
	gcomm.updateHeliState = fillHeliState;
	gcomm.setPeriodicState = setPeriodicState;
	gcomm.error = error_display;
	gcomm.reset = reset;
	gcomm.configureBluetooth = configureBluetooth;
	gcomm.updateControlParams = updateControlParams;
	gcomm.updateTrimMode = updateTrimMode;
	gcomm.setLight = setLightSensor;
	gcomm.verbose = 0;
	gcomm.debug_channel = -1;
	gcomm.takeoffHeight_mm = 0;
	gcomm.landingHeight_mm = 180;
	gcomm.landingStep_mm = 5;
	buildCapacities(gcomm.capacities); // initialise the list of capacities

	// Channels are defined in the global variables
#ifdef LINUX
	sbChannelCreateSocketUDP(gcomm.channel+CHANNEL_0_ID,NULL,5123);
	sbChannelCreateSocketUDP(gcomm.channel+CHANNEL_1_ID,NULL,5124);
#endif
#ifdef WIN32
	sbChannelCreateWinsockUDP(gcomm.channel+CHANNEL_0_ID,NULL,5123);
	sbChannelCreateWinsockUDP(gcomm.channel+CHANNEL_1_ID,NULL,5124);
#endif
	sbChannelOpen(gcomm.channel+CHANNEL_0_ID);
	sbChannelOpen(gcomm.channel+CHANNEL_1_ID);
	printf("Opened channel on UDP ports 5123 and 5124\n");

	startCommThread();

	simulation_loop();

	stopCommThread();

	sbChannelDestroy(gcomm.channel+CHANNEL_0_ID);
	sbChannelDestroy(gcomm.channel+CHANNEL_1_ID);
	return 0;
};

