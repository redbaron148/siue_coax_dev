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
#ifndef SB_CONST_H
#define SB_CONST_H


/** \file sbconst.h
All the constants needed for the communications
The constant related to the contents are not there to 
keep this 16bits. See sbstate.h for the 16bits constants
See sbapi.h for the 32bits constants
*/

/** Connect message */
#define SB_MSGID_CONNECT     0x01u
/** Disconnect message */
#define SB_MSGID_DISCONNECT  0x02u
/** Sensorlist message */
#define SB_MSGID_SENSORLIST  0x03u
/** Cfg_oavoid message */
#define SB_MSGID_CFG_OAVOID  0x04u
/** Cfg_comm message */
#define SB_MSGID_CFG_COMM    0x05u
/** Cfg_control message */
#define SB_MSGID_CFG_CONTROL 0x06u
/** Cfg_timeout message */
#define SB_MSGID_CFG_TIMEOUT 0x07u
/** State message */
#define SB_MSGID_STATE       0x08u
/** Set_navmode message */
#define SB_MSGID_SET_NAVMODE 0x09u
/** Set_control message */
#define SB_MSGID_SET_CONTROL 0x0Au
/** Keep_alive message */
#define SB_MSGID_KEEP_ALIVE  0x0Bu
/** Raw_command message */
#define SB_MSGID_RAW_COMMAND 0x0Cu
/** Cfg_raw_cmd message */
#define SB_MSGID_CFG_RAW_CMD 0x0Du
/** String message */
#define SB_MSGID_STRING      0x0Eu
/** Reset message */
#define SB_MSGID_RESET       0x0Fu
/** Cfg_commloop message */
#define SB_MSGID_CFG_COMMLOOP  0x10u
/** Trim mode message */
#define SB_MSGID_TRIMMODE  0x11u
/** Control parameters message */
#define SB_MSGID_CTRLPARM  0x12u
/** Set control message, including timestamp */
#define SB_MSGID_SET_CONTROL_WITH_TIMESTAMP  0x13u
/** Request versionning information */
#define SB_MSGID_GET_VERSION  0x14u
/** Set bluetooth configuration */
#define SB_MSGID_CFG_BLUETOOTH  0x15u
/** Used to transmit of 6 DOF pose */
#define SB_MSGID_6DOF_POSE  0x16u
/** Used to get/set custom data on the dsPIC, mostly to pass them
 * between Xbee and serial*/
#define SB_MSGID_CUSTOM  0x17u

/** Configure the light of the coaxspeed */
#define SB_MSGID_SET_LIGHT  0x18u

/** Mask for the message id part of the message */
#define SB_MSGID_MASK        0x3Fu
/** Reqack flag message */
#define SB_MSGID_REQACK      0x40u
/** Reply flag message */
#define SB_MSGID_REPLY       0x80u

/**
 * Any message sent from the platform is 
 * or'ed with SB_MSGID_REPLY 
 * */


/** No obstacle avoidance */
#define SB_OA_NONE 0x00u
/** Horizontal obstacle avoidance */
#define SB_OA_HORIZONTAL 0x01u
/** Vertical obstacle avoidance */
#define SB_OA_VERTICAL 0x02u
/** Height above obstacles (in mm) at which the vertical obstacle avoidance 
 * has an influence */
#define SB_OA_MAX_ALT 15
/** Maximum distance (in cm) at which the obstacle avoidance 
 * has an influence */
#define SB_OA_MAX_DIST 150
/** Maximum allowed pitch/roll correction from the obstacle avoidance
 * in hundreth of degrees */
#define SB_OA_MAX_CORRECTION 20

/** Communication on request only */
#define SB_COM_ONREQUEST 0x00u
/** Continuous communication */
#define SB_COM_CONTINUOUS 0x01u

/** No control */
#define SB_CTRL_NONE 0x00
/** Position control */
#define SB_CTRL_POS 0x01
/** Relative set point control (wrt range for instance) */
#define SB_CTRL_REL 0x02
/** Velocity control */
#define SB_CTRL_VEL 0x03
/** Acceleration/Force control, currently only used for thrust */
#define SB_CTRL_FORCE 0x04
/** Manual control */
#define SB_CTRL_MANUAL 0x08
#define SB_CTRL_MANUAL_MASK 0x07

/** Step velocity profile for servos in raw mode */
#define SB_RAWPROFILE_STEP 0x00u
/** Ramp velocity profile for servos in raw mode */
#define SB_RAWPROFILE_RAMP 0x01u

/** Navigation mode: STOP */
#define SB_NAV_STOP 0x00u
/** Navigation mode: IDLE */
#define SB_NAV_IDLE 0x01u
/** Navigation mode: TAKEOFF */
#define SB_NAV_TAKEOFF 0x02u
/** Navigation mode: LAND */
#define SB_NAV_LAND 0x03u
/** Navigation mode: HOVER */
#define SB_NAV_HOVER 0x04u
/** Navigation mode: CTRLLED */
#define SB_NAV_CTRLLED 0x05u
/** Navigation mode: SINK */
#define SB_NAV_SINK 0x06u
/** Navigation mode: RAW */
#define SB_NAV_RAW 0x07u
/** Navigation mode: MANUAL (unused) */
#define SB_NAV_MANUAL 0x08u

/** Error code in acknowledgement message: no error */
#define SB_REPLY_OK    0x00
/** Error code in acknowledgement message: invalid navigation mode */
#define SB_REPLY_INVALID_NAVMODE 0xfd
/** Error code in acknowledgement message: unknown message id or request */
#define SB_REPLY_UNKNOWN 0xfe
/** Error code in acknowledgement message: some error */
#define SB_REPLY_ERROR 0xff
/** Error code in acknowledgement message: could not decode message */
#define SB_REPLY_DECODE_FAILURE 0xfc
/** Error code in acknowledgement message: request not accepted because system
 * is busy with another connection*/
#define SB_REPLY_BUSY 0xfb
/** Error code in acknowledgement message: too early to do this request */
#define SB_REPLY_TOO_EARLY 0xfa

/** Maximum message length in SBStringMessage */
#define SB_STRING_MESSAGE_LENGTH 64

/** Maximum compile time string length in SBVersionStatus */
#define SB_COMPILE_TIME_LENGTH 16
/** Maximum imu version  length in SBVersionStatus */
#define SB_IMU_VERSION_LENGTH 16


/** Identification of the range measurement */
#define SB_RANGE_FRONT 0
#define SB_RANGE_RIGHT 1
#define SB_RANGE_LEFT 2
#define SB_RANGE_BACK 3

/** Id of trim modes */
#define SB_TRIM_FROM_RC 0
#define SB_TRIM_SOFTWARE 1

/** If for RC channels */
#define SB_RC_THROTTLE 0
#define SB_RC_THROTTLE_TRIM 1
#define SB_RC_YAW 2
#define SB_RC_YAW_TRIM 3
#define SB_RC_ROLL 4
#define SB_RC_ROLL_TRIM 5
#define SB_RC_PITCH 6
#define SB_RC_PITCH_TRIM 7

/** Duration of startup time */
#define MIN_TIME_IN_IDLE 1000

#define DEBUG_CHANNEL_NONE -1
#define DEBUG_CHANNEL_GUMSTIX 0
#define DEBUG_CHANNEL_BLUETOOTH 1


/** State of the coax speed sensor, availability, and measurement status */
#define COAXSPEED_AVAILABLE 1
#define COAXSPEED_VALID_MEASUREMENT 2

/** Error flags */
#define SB_FLAG_IMUCRASH 0x01
#define SB_FLAG_LOWPOWER 0x02
#define SB_FLAG_RCLOST 0x04
#define SB_FLAG_MANUAL 0x40
#define SB_FLAG_KILLED 0x80


#endif /* SB_CONST_H */
