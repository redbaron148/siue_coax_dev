package com.skybotix.sbapi;

public class SBConsts {
    /** \file sbconst.h
      All the constants needed for the communications
      The constant related to the contents are not there to 
      keep this 16bits. See sbstate.h for the 16bits constants
      See sbapi.h for the 32bits constants
      */

    /** Connect message */
    final static public int SB_MSGID_CONNECT = 0x01;
    /** Disconnect message */
    final static public int SB_MSGID_DISCONNECT = 0x02;
    /** Sensorlist message */
    final static public int SB_MSGID_SENSORLIST = 0x03;
    /** Cfg_oavoid message */
    final static public int SB_MSGID_CFG_OAVOID = 0x04;
    /** Cfg_comm message */
    final static public int SB_MSGID_CFG_COMM = 0x05;
    /** Cfg_control message */
    final static public int SB_MSGID_CFG_CONTROL = 0x06;
    /** Cfg_timeout message */
    final static public int SB_MSGID_CFG_TIMEOUT = 0x07;
    /** State message */
    final static public int SB_MSGID_STATE = 0x08;
    /** Set_navmode message */
    final static public int SB_MSGID_SET_NAVMODE = 0x09;
    /** Set_control message */
    final static public int SB_MSGID_SET_CONTROL = 0x0A;
    /** Keep_alive message */
    final static public int SB_MSGID_KEEP_ALIVE = 0x0B;
    /** Raw_command message */
    final static public int SB_MSGID_RAW_COMMAND = 0x0C;
    /** Cfg_raw_cmd message */
    final static public int SB_MSGID_CFG_RAW_CMD = 0x0D;
    /** String message */
    final static public int SB_MSGID_STRING = 0x0E;
    /** Reset message */
    final static public int SB_MSGID_RESET = 0x0F;
    /** Cfg_commloop message */
    final static public int SB_MSGID_CFG_COMMLOOP = 0x10;
    /** Trim mode message */
    final static public int SB_MSGID_TRIMMODE = 0x11;
    /** Control parameters message */
    final static public int SB_MSGID_CTRLPARM = 0x12;
    /** Set control message, including timestamp */
    final static public int SB_MSGID_SET_CONTROL_WITH_TIMESTAMP = 0x13;
    /** Request versionning information */
    final static public int SB_MSGID_GET_VERSION = 0x14;
    /** Set bluetooth configuration */
    final static public int SB_MSGID_CFG_BLUETOOTH = 0x15;
    /** Used to transmit of 6 DOF pose */
    final static public int SB_MSGID_6DOF_POSE = 0x16;
    /** Used to get/set custom data on the dsPIC, mostly to pass them
     * between Xbee and serial*/
    final static public int SB_MSGID_CUSTOM = 0x17;

    /** Configure the light of the coaxspeed */
    final static public int SB_MSGID_SET_LIGHT = 0x18;

    /** Mask for the message id part of the message */
    final static public int SB_MSGID_MASK = 0x3F;
    /** Reqack flag message */
    final static public int SB_MSGID_REQACK = 0x40;
    /** Reply flag message */
    final static public int SB_MSGID_REPLY = 0x80;

    /**
     * Any message sent from the platform is 
     * or'ed with SB_MSGID_REPLY 
     * */


    /** No obstacle avoidance */
    final static public int SB_OA_NONE = 0x00;
    /** Horizontal obstacle avoidance */
    final static public int SB_OA_HORIZONTAL = 0x01;
    /** Vertical obstacle avoidance */
    final static public int SB_OA_VERTICAL = 0x02;
    /** Height above obstacles (in mm) at which the vertical obstacle avoidance 
     * has an influence */
    final static public int SB_OA_MAX_ALT = 15;
    /** Maximum distance (in cm) at which the obstacle avoidance 
     * has an influence */
    final static public int SB_OA_MAX_DIST = 150;
    /** Maximum allowed pitch/roll correction from the obstacle avoidance
     * in hundreth of degrees */
    final static public int SB_OA_MAX_CORRECTION = 20;

    /** Communication on request only */
    final static public int SB_COM_ONREQUEST = 0x00;
    /** Continuous communication */
    final static public int SB_COM_CONTINUOUS = 0x01;

    /** No control */
    final static public int SB_CTRL_NONE = 0x00;
    /** Position control */
    final static public int SB_CTRL_POS = 0x01;
    /** Relative set point control (wrt range for instance) */
    final static public int SB_CTRL_REL = 0x02;
    /** Velocity control */
    final static public int SB_CTRL_VEL = 0x03;
    /** Acceleration/Force control, currently only used for thrust */
    final static public int SB_CTRL_FORCE = 0x04;
    /** Manual control */
    final static public int SB_CTRL_MANUAL = 0x08;
    final static public int SB_CTRL_MANUAL_MASK = 0x07;

    /** Step velocity profile for servos in raw mode */
    final static public int SB_RAWPROFILE_STEP = 0x00;
    /** Ramp velocity profile for servos in raw mode */
    final static public int SB_RAWPROFILE_RAMP = 0x01;

    /** Navigation mode: STOP */
    final static public int SB_NAV_STOP = 0x00;
    /** Navigation mode: IDLE */
    final static public int SB_NAV_IDLE = 0x01;
    /** Navigation mode: TAKEOFF */
    final static public int SB_NAV_TAKEOFF = 0x02;
    /** Navigation mode: LAND */
    final static public int SB_NAV_LAND = 0x03;
    /** Navigation mode: HOVER */
    final static public int SB_NAV_HOVER = 0x04;
    /** Navigation mode: CTRLLED */
    final static public int SB_NAV_CTRLLED = 0x05;
    /** Navigation mode: SINK */
    final static public int SB_NAV_SINK = 0x06;
    /** Navigation mode: RAW */
    final static public int SB_NAV_RAW = 0x07;
    /** Navigation mode: MANUAL (unused) */
    final static public int SB_NAV_MANUAL = 0x08;

    /** Error code in acknowledgement message: no error */
    final static public int SB_REPLY_OK = 0x00;
    /** Error code in acknowledgement message: invalid navigation mode */
    final static public int SB_REPLY_INVALID_NAVMODE = 0xfd;
    /** Error code in acknowledgement message: unknown message id or request */
    final static public int SB_REPLY_UNKNOWN = 0xfe;
    /** Error code in acknowledgement message: some error */
    final static public int SB_REPLY_ERROR = 0xff;
    /** Error code in acknowledgement message: could not decode message */
    final static public int SB_REPLY_DECODE_FAILURE = 0xfc;
    /** Error code in acknowledgement message: request not accepted because system
     * is busy with another connection*/
    final static public int SB_REPLY_BUSY = 0xfb;
    /** Error code in acknowledgement message: too early to do this request */
    final static public int SB_REPLY_TOO_EARLY = 0xfa;

    /** Maximum message length in SBStringMessage */
    final static public int SB_STRING_MESSAGE_LENGTH = 64;

    /** Maximum compile time string length in SBVersionStatus */
    final static public int SB_COMPILE_TIME_LENGTH = 16;
    /** Maximum imu version  length in SBVersionStatus */
    final static public int SB_IMU_VERSION_LENGTH = 16;


    /** Identification of the range measurement */
    final static public int SB_RANGE_FRONT = 0;
    final static public int SB_RANGE_RIGHT = 1;
    final static public int SB_RANGE_LEFT = 2;
    final static public int SB_RANGE_BACK = 3;

    /** Id of trim modes */
    final static public int SB_TRIM_FROM_RC = 0;
    final static public int SB_TRIM_SOFTWARE = 1;

    /** If for RC channels */
    final static public int SB_RC_THROTTLE = 0;
    final static public int SB_RC_THROTTLE_TRIM = 1;
    final static public int SB_RC_YAW = 2;
    final static public int SB_RC_YAW_TRIM = 3;
    final static public int SB_RC_ROLL = 4;
    final static public int SB_RC_ROLL_TRIM = 5;
    final static public int SB_RC_PITCH = 6;
    final static public int SB_RC_PITCH_TRIM = 7;

    /** Duration of startup time */
    final static public int MIN_TIME_IN_IDLE = 1000;

    final static public int DEBUG_CHANNEL_NONE = -1;
    final static public int DEBUG_CHANNEL_GUMSTIX = 0;
    final static public int DEBUG_CHANNEL_BLUETOOTH = 1;


    /** State of the coax speed sensor, availability, and measurement status */
    final static public int COAXSPEED_AVAILABLE = 1;
    final static public int COAXSPEED_VALID_MEASUREMENT = 2;

    /** Error flags */
    final static public int SB_FLAG_IMUCRASH = 0x01;
    final static public int SB_FLAG_LOWPOWER = 0x02;
    final static public int SB_FLAG_RCLOST = 0x04;
    final static public int SB_FLAG_MANUAL = 0x40;
    final static public int SB_FLAG_KILLED = 0x80;


}
