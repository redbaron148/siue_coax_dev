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
#ifndef SKYBOTICS_COM_STATE_H
#define SKYBOTICS_COM_STATE_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Skybotics message definition interface
 * Author: C. Pradalier
 *
 * */

typedef struct  {
	unsigned char errorFlags;
	unsigned char padding;
	unsigned short content[2];
	unsigned long timeStamp;
	unsigned short controlTimeout;
	unsigned short watchdogTimeout;
	unsigned short commFrequency;
	struct {
		unsigned char navigation;
		unsigned char communication;
		unsigned char oavoid;
		unsigned char rollAxis;

		unsigned char pitchAxis;
		unsigned char yawAxis;
		unsigned char altAxis;
	} mode;

	struct {
		// Desired mode
		unsigned char rollAxis;
		unsigned char pitchAxis;
		unsigned char yawAxis;
		unsigned char altAxis;
		// Desired value
		signed short roll;
		signed short pitch;
		signed short yaw;
		signed short altitude;
		// Raw
		signed short motor1;
		signed short motor2;
		signed short servo1;
		signed short servo2;
	} setpoint;

	struct {
		// Control output, read by the motor controller
		signed short roll;
		signed short pitch;
		signed short yaw;
		signed short altitude;
	} control;

	signed short roll, pitch, yaw;
	signed short gyro[3];
	signed short accel[3];
	signed short magneto[3];
	unsigned short imutemp;
	signed short pressure;
	signed short zrange;
	signed short zfiltered;
	unsigned short hranges[4];
	signed short xrel, yrel;
	unsigned short battery;

	signed short o_attitude[3];
	signed short o_altitude;
	signed short o_tol;
	signed short o_xy[2];
	signed short o_oavoid[2];

	signed short rcChannel[8];
	unsigned char rawSpeedProfile[2];

    struct {
        unsigned char state;
        unsigned char light;
        signed short vel_x;
        signed short vel_y;
    } coaxspeed;

} SBHeliStateRaw;


#include "sbmessage.h"

/** 
 * Data selection flags for skybotics platform communication
 * To be used only for direct message composition
 * */
	

#define SBC_MODES		  0	/* System mode (nav, comm, ...) */
#define SBC_TIMESTAMP	  1	/* System timestamp */
#define SBC_RPY			  2	/* Roll, pitch, yaw from IMU */
#define SBC_GYRO		  3	/* Roll, pitch, yaw rate from IMU */
#define SBC_ACCEL		  4	/* Acceleration from IMU */
#define SBC_MAGNETO		  5	/* Magnetic field vector from IMU */
#define SBC_IMUTEMP		  6	/* Temperature from IMU */
#define SBC_ALTITUDE	  7	/* Range to the floor, and filtered altitude */
#define SBC_PRESSURE	  8	/* Measurement from pressure sensor */
#define SBC_HRANGES		  9	/* Horizontal ranges to obstacles */
#define SBC_XY_REL		 10	/* Distance to closest object */
#define SBC_BATTERY		 11	/* Battery status */
#define SBC_TIMEOUT		 12	/* Battery status */
#define SBC_O_ATTITUDE	 13	/* Output of Attitude control */
#define SBC_O_ALTITUDE	 14	/* Output of Altitude control */
#define SBC_O_TOL		 15	/* Output of TakeOff/Landing control */
#define SBC_O_XY		 16	/* Output of XY control */
#define SBC_O_OAVOID	 17	/* Output of Obstacle avoidance control */
#define SBC_CHANNELS	 18	/* Output of remote control */
#define SBC_COAXSPEED 	 19	/* Status of the coaxspeed */

/* 20 ... 26: reserved */

/* Combined flags for convenience */
#define SBC_ALL			 27
#define SBC_IMU_ALL		 28
#define SBC_RANGES_ALL	 29
#define SBC_ALTITUDE_ALL 30
#define SBC_OUTPUT_ALL	 31




#ifdef PIC30
int sbAddOneContent(unsigned short content[2], int what);
#else
/* Used to mark the last content in sbAddContent only */
#define SBC_END_OF_CONTENT -1
int sbAddContent(unsigned short content[2], int what,  ...);
#endif

int sbTestContent(unsigned short content[2], unsigned int what);


int sbStateEncode(SBSerialisedMessage *msg, unsigned char handle, const SBHeliStateRaw* state);
#ifndef PIC30
int sbStateDecode(const SBSerialisedMessage *msg, SBHeliStateRaw* state);
#endif

/**   Provide a string representation of constant SB_CTRL_xxxx   
 *   defined in sbconst.h. Return NULL for invalid values.  */
const char* sbCtrlModeString(unsigned char c);
/**   Provide a string representation of constant SB_COM_xxxx   
 *   defined in sbconst.h. Return NULL for invalid values.  */
const char* sbCommModeString(unsigned char c);
/**   Provide a string representation of constant SB_OA_xxxx   
 *   defined in sbconst.h. Return NULL for invalid values.  */
const char* sbOAModeString(unsigned char c);
/**   Provide a string representation of constant SB_NAV_xxxx   
 *   defined in sbconst.h. Return NULL for invalid values.  */
const char* sbNavModeString(unsigned char c);
#ifdef SBC_HAS_IO

#include <stdio.h>

void sbStateRawPrint(FILE *fp,SBHeliStateRaw *hs);
void sbContent16bPrint(FILE *fp, const unsigned short content[2]);

#endif // SBC_HAS_IO

#ifdef __cplusplus
}
#endif

#endif // SKYBOTICS_COM_STATE_H
