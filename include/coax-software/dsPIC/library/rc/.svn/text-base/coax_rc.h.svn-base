/*************************************************************
*
* Low-level sensor and control library for the Skybotix AG
* helicopters, and particularly COAX
*
* Developed by:
*  * Samir Bouabdallah samir.bouabdallah@skybotix.ch
*  * Cedric Pradalier: cedric.pradalier@skybotix.ch
*  * Gilles Caprari: gilles.capraru@skybotix.ch
*  * Thomas Baumgartner
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
* Description:This software manages the RC receiver of the CoaX
*************************************************************/
#ifndef __coax_rc_H_
#define __coax_rc_H_

extern float RC_PITCH;
extern float RC_ROLL;
extern float RC_THROTTLE;
extern float RC_YAW;
extern float RC_PITCH_TRIM;
extern float RC_ROLL_TRIM;
extern float RC_THROTTLE_TRIM;
extern float RC_YAW_TRIM;


// RC24 Receiver Globals. ----------------------------------------------------
extern unsigned char rcRawData[16];
extern unsigned int rcData[8];
extern int activeChannelIndex;
extern unsigned char channelList[3];
extern float rcValue[8];
extern unsigned int ERROR_Level1_COUNTER;
extern unsigned int ERROR_Level2_COUNTER;



/****** RC CHANNELS ******/
#define WK2401_RAW_PITCH           rcData[0]
#define WK2401_RAW_PITCH_TRIM      rcData[1]
#define WK2401_RAW_ROLL            rcData[2]
#define WK2401_RAW_ROLL_TRIM       rcData[3]
#define WK2401_RAW_THROTTLE        rcData[4]
#define WK2401_RAW_THROTTLE_TRIM   rcData[5]
#define WK2401_RAW_YAW             rcData[6]
#define WK2401_RAW_YAW_TRIM        rcData[7]

#define WK2402_RAW_PITCH           rcData[0]
#define WK2402_RAW_PITCH_TRIM      rcData[1]
#define WK2402_RAW_ROLL            rcData[2]
#define WK2402_RAW_ROLL_TRIM       rcData[3]
#define WK2402_RAW_THROTTLE        rcData[4]
#define WK2402_RAW_THROTTLE_TRIM   rcData[5]
#define WK2402_RAW_YAW             rcData[6]
#define WK2402_RAW_YAW_TRIM        rcData[7]

typedef enum {
    RC_UNDEF=0, 
    RC_WK2401, 
    RC_WK2402,
    RC_NUMTYPES
} RC_TYPE;

typedef enum {
    RC_LOWLEVEL_INIT=0,
    RC_LOWLEVEL_SEARCHING,
    RC_LOWLEVEL_RUNNING,

    /*** Reset state, to avoid blocking wait ***/
    RC_LOWLEVEL_RESET1=0x80,
    RC_LOWLEVEL_RESET1WAIT,
    RC_LOWLEVEL_RESET2,
    RC_LOWLEVEL_RESET2WAIT,
} RC_LOWLEVEL_STATE;

typedef enum {RCSCPrepare, RCSCTune, RCSCWaitResponse, 
    RCSCReceivedResponse, RCSCCompleted, RCSCReady} RCSC_STATE;

/****** RC CHANNELS SCALING CONSTANTS ******/

#define PITCH_SLOPE            0.0025      // RC 2.4Ghz to -1 to +1 values
#define PITCH_SHIFT            1.3065
#define PITCH_TRIM_SLOPE       0.0026      
#define PITCH_TRIM_SHIFT       1.3814

#define ROLL_SLOPE             0.0024      // RC 2.4Ghz to -1 to +1 values      
#define ROLL_SHIFT             1.2449
#define ROLL_TRIM_SLOPE        0.0025      
#define ROLL_TRIM_SHIFT        1.3061

#define THROTTLE_SLOPE         0.0013   // RC 2.4Ghz to 0 to +1 values   
#define THROTTLE_SHIFT         0.1186
#define THROTTLE_TRIM_SLOPE    0.0013      
#define THROTTLE_TRIM_SHIFT    0.1758

#define YAW_SLOPE              0.0025   // RC 2.4Ghz to -1 to +1 values      
#define YAW_SHIFT              1.3441
#define YAW_TRIM_SLOPE         0.0026      
#define YAW_TRIM_SHIFT         1.346


int RCInitReception(int blocking);
int RCIsReady(void);
void RCSetType(RC_TYPE type);
RC_TYPE RCGetType();
RC_LOWLEVEL_STATE RCGetState();

extern RC_LOWLEVEL_STATE rcState;
extern RCSC_STATE rcSearchState;
#if 0
// Debug variable
extern unsigned char NumberOfFoundChannels;
extern unsigned char Channel;
extern unsigned char global_irq_status;
#endif 

#endif
