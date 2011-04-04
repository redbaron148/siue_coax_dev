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
#ifndef SB_API_H
#define SB_API_H


#ifdef __cplusplus
extern "C" {
#endif

#include "sbconst.h"
/** 
 * \file sbapi.h Data selection flags for skybotics platform communication
 * This is not defined in sbconst.h to keep the header 16bit only
 *  General remarks: All the functions below return an int. The
 * semantic is (as usual in C programming) 0 for no error. Any other number
 * corresponds to an error situation. TODO: document error codes
 * */

 /** System mode (nav, comm, ...) */
#define SBS_MODES		 0x00000001ul	
 /** System timestamp */
#define SBS_TIMESTAMP	 0x00000002ul	
 /** Roll, pitch, yaw from IMU */
#define SBS_RPY			 0x00000004ul	
 /** Roll, pitch, yaw rate from IMU */
#define SBS_GYRO		 0x00000008ul	
 /** Acceleration from IMU */
#define SBS_ACCEL		 0x00000010ul	
 /** Magnetic field vector from IMU */
#define SBS_MAGNETO		 0x00000020ul	
 /** Temperature from IMU */
#define SBS_IMUTEMP		 0x00000040ul	
 /** Range to the floor, and filtered altitude */
#define SBS_ALTITUDE	 0x00000080ul	
 /** Altitude from pressure sensor */
#define SBS_PRESSURE	 0x00000100ul	
 /** Horizontal ranges to obstacles */
#define SBS_HRANGES		 0x00000200ul	
 /** Distance to closest object */
#define SBS_XY_REL		 0x00000400ul	
 /** Battery status */
#define SBS_BATTERY		 0x00000800ul	
 /** Timeout currently used */
#define SBS_TIMEOUT		 0x00001000ul	
 /** Output of Attitude control */
#define SBS_O_ATTITUDE	 0x00002000ul	
 /** Output of Altitude control */
#define SBS_O_ALTITUDE	 0x00004000ul	
 /** Output of TakeOff/Landing control */
#define SBS_O_TOL		 0x00008000ul	
 /** Output of XY control */
#define SBS_O_XY		 0x00010000ul	
 /** Output of Obstacle avoidance control */
#define SBS_O_OAVOID	 0x00020000ul	

 /** Remote control channels */
#define SBS_CHANNELS	 0x00040000ul	

 /** Coax speed status */
#define SBS_COAXSPEED 	 0x00080000ul	

/** Combined flags for convenience */
#define SBS_ALL			 0x000FFFFFul
#define SBS_IMU_ALL		 0x0000007Cul
#define SBS_RANGES_ALL	 0x00000280ul
#define SBS_ALTITUDE_ALL 0x00000180ul
#define SBS_OUTPUT_ALL	 0x0003E000ul




/** Structure representing the helicopter state as transmitted by the low-level
 * but scaled to useful units. 
 * \note Only SI unit used here: rad, rad/s, m, m/s  */
typedef struct  {
	/** Error status set by the helicopter */
	unsigned char errorFlags;
	/** 
	 *	 Affected content in this data structure (
	 *   Use AND with the SBS_... flags above to check the content  
	 *   e.g: if (state.content & SBS_RPY) {  
	 *  			compute_odo(state.roll,state.pitch,state.yaw);  
	 *  		}  
	 *   This content should correspond to what has been configured in  
	 *   sbConfigureComm or requested in sbRequestState  */
	unsigned long content;
	/** timestamp of the last update, in ms since the initialisation of the
	 * helicopter. */
	unsigned long timeStamp;
	/** current control timeout (for sending command in SB_NAV_CTRLLED mode) */
	unsigned short controlTimeout;
	/** current comm timeout, to bring the helicopter back to safety is
	 * communication is not maintained. */
	unsigned short watchdogTimeout;
	/** Various bit field to represent the system configuration*/
	struct {
		/** Navigation mode: SB_NAV_... */
		unsigned char navigation;
		/** Communication mode: SB_COM_... */
		unsigned char communication;
		/** Obstacle avoidance mode: or of SB_OA_... */
		unsigned char oavoid;
		/** Control mode for roll axis: SB_CTRL_... */
		unsigned char rollAxis;
		/** Control mode for pitch axis: SB_CTRL_... */
		unsigned char pitchAxis;
		/** Control mode for yaw axis: SB_CTRL_... */
		unsigned char yawAxis;
		/** Control mode for altitude axis: SB_CTRL_... */
		unsigned char altAxis;
	} mode;

	/** Current helicopter attitude */
	float roll, pitch, yaw;
	/** GYRO data */
	float gyro[3];
	/** Accelerometer data */
	float accel[3];
	/** Magnetometer data */
	float magneto[3];
	/** Temperature measured by IMU */
	float imutemp;
	/** Range measurement in the vertical direction */
	float zrange;
	/** Filtered altitude, as used by the altitude control in POS mode */
	float zfiltered;
	/** Output of pressure sensor */
	float pressure;
	/** Range measurements in the horizontal plane. Sensor placement is
	 * platform dependent */
	float hranges[4];
	/** Distance to closest obstacle (if implemented) */
	float xrel, yrel;
	/** Battery voltage */
	float battery;
	/** Output of the remote control channel, normalised to [-1,1] */
	float rcChannel[8];

    struct {
        unsigned char state; /** see sbconst.h COAXSPEED_* constants */
        unsigned char light;
        float vel_x, vel_y;
    } coaxspeed;

	/* symbols below may be suppressed in future version of the library */

	/** Output of attitude control (semantic unclear) */
	float o_attitude[3];
	/** Output of altitude control, i.e. thrust to keep the helicopter affloat  */
	float o_altitude;
	/** Output of take-off/landing control (semantic unclear) */
	unsigned short o_tol;
	/** ??? (semantic unclear) */
	float o_xy[2];
	/** ??? (semantic unclear) */
	float o_oavoid[2];
} SBHeliState;
		
/**   Abstract type declaring the control context, the communication channel  
 *   and other implementation specific data. This is only defined in this header
 *   to permit an implementation in static memory 
 *   Do not rely on a specific implementation of this type **/
#include "sbchannel.h"
struct SBControlContext {
	SBChannel channel;
	int handle;
	int ackMode;
	void (*stateCallback)(SBHeliState *state, void *userData);
	void *userData;
} ;

/** \struct representing the trim settings  */
struct SBTrimMode {
	/** Trim mode, can be SB_TRIM_FROM_RC or SB_TRIM_SOFTWARE */
	unsigned char trimMode; 
	/** Trim position for the roll axis */
	float rollTrim;
	/** Trim position for the pitch axis */
	float pitchTrim;
};

/** \struct representing the control parameters  */
struct SBControlParameters {
	/** Base thrust around which the control operates */
	float baseThrust;
	/** Difference of operating point between up and down motors */
	float yawOffset;
	/** Altitude gain: Kp*/
	float altitudeKp;
	/** Altitude gain: Ki*/
	float altitudeKi;
	/** Altitude gain: Kd*/
	float altitudeKd;

	/** Yaw gain: Kp*/
	float yawKp;
	/** Yaw gain: Ki*/
	float yawKi;
	/** Yaw gain: Kd*/
	float yawKd;
};


#include "sbversion.h"


#ifdef WIN32
/**   Open a serial communication channel to the helicopter via bluetooth
 * (WIN32 only).   */
int sbOpenBluetoothCommunication(struct SBControlContext *control,
		const char *port, unsigned int speed);
/**   Open a network communication channel to the helicopter.     This may be
 * used in simulation or possibly to communicate via an on-board
 * network-enabled processor.   */
int sbOpenWinsockCommunication(struct SBControlContext *control,
		const char *hostname, unsigned int port);
#endif


#ifdef LINUX
/**   Open a serial communication channel to the helicopter.      Under linux,
 *   device may be (for instance) "/dev/tty0" or "/dev/rfcomm0" for
 *   bluetooth, and baudrate is a define symbol as defined in bit/termios.h
 *   (e.g.     B115200).  */
int sbOpenSerialCommunication(struct SBControlContext *control,
		const char *device, unsigned int baudrate, int rtscts);


/**   Open a network communication channel to the helicopter.     This may be
 * used in simulation or possibly to communicate via an on-board
 * network-enabled processor.   */
int sbOpenSocketCommunication(struct SBControlContext *control,
		const char *hostname, unsigned int port);
#endif

/**   Flush the communication channel   */
int sbFlushCommunication(struct SBControlContext *control);

/**   Close the communication channel and release any resource if needed  */
int sbCloseCommunication(struct SBControlContext *control);

/**   Lock the communication channel to prevent concurrent access from
 * different     threads. Currently, this will only work on POSIX systems.  */
int sbLockCommunication(struct SBControlContext *control);

/**   Unlock the communication channel  */
int sbUnlockCommunication(struct SBControlContext *control);

/** Open a connection to the helicopter. This is required in order to get the
 * permissions to change parameters or control the helicopter. For receiving
 * data, there is no need to connect  */
int sbConnect(struct SBControlContext *control);

/** Release the connection to the helicopter */
int sbDisconnect(struct SBControlContext *control);

/**   This function illustrates the spirit of this API.  
 *   At any one time, only one process/thread should use the control  
 *   channel.   
 *   In synchronous mode, the dialogue is only question/response  
 *   When the communication is continuous, the helicopter may send state  
 *   information at any time. When a question/response mode is started, some  
 *   state information may arrive in-between. This callback can be setup to  
 *   handle this piece of data in this case, instead of discarding it.  
 *   Example:  
 *    
 *   <pre>
 *   // Global variable to store the state, for illustration  
 *   SBHeliState state;  
 *    
 *   void * receive_thread(void *) {  
 *  	 ...  
 *  	 while (1) {  
 *  		sbLockCommunication(&control);  
 *  		sbReceiveState(control,&state);  
 *  		sbUnlockCommunication(&control);  
 *  	 }  
 *  	}  
 *    
 *  	void recCB(SBHeliState *rstate, void *user_data) {  
 *  		memcpy(&state,&rstate,sizeof(SBHeliState);  
 *  	}  
 *    
 *  	int main() {  
 *  		struct SBControlContext control;  
 *  		unsigned long sensors = SBS_ALL;  
 *  		unsigned long content = SBS_ALL;  
 *  		sbOpenSerialCommunication(&control,"/dev/rfcomm0",B115200)  
 *  		sbConnect(&control);  
 *  		sbRegisterStateCallback(&control,recCB,NULL);  
 *  		sbGetSensorList(&control, &sensors);  
 *  		sbConfigureOAMode(&control,SB_OA_NONE);  
 *  		sbConfigureControl(&control,SB_CTRL_NONE,SB_CTRL_NONE,  
 *  				SB_CTRL_POS, SB_CTRL_POS, SB_CTRL_VEL);  
 *  		// In order to request only the data available on this platform  
 *  		sbConfigureComm(SB_COM_CONTINUOUS,30,content & sensors);  
 *  		start receive_thread()  
 *  		while (1) {  
 *  			wait something  
 *  			sbLockCommunication(&control);  
 *  			// Any state received here will be handle by the callback  
 *  			sbSetNavMode(&control,SB_NAV_HOVER);  
 *  			sbUnlockCommunication(&control);  
 *  		}  
 *  	}   
 *  	</pre>
 *  	**/
int sbRegisterStateCallback(struct SBControlContext *control, 
	void (*stateCallback)(SBHeliState *state, void *userData),
	void *userData);

/**   Returns a content mask, where *list is a OR of the SBS_xxxx flags which
 * make     sense on this platform.  */
int sbGetSensorList(struct SBControlContext *control, unsigned long *list);

/**   Configure the communication so that control requests (sbSetNavMode,
 * sbKeepAlive, sbSetControl) requires an acknowledgement     from the
 * helicopter. If requestAck is zero, all control functions      will not
 * confirm reception of their commands. With bluetooth communication,     this
 * can improve the communication rate. If requestAck is non zero, the
 * control request will be followed by an acknowledgement.  
 * */
int sbConfigureAckMode(struct SBControlContext *control, unsigned int requestAck);

/**   Set commloop verbosity, for debug mode  
 *   */
int sbConfigureCommLoop(struct SBControlContext *control, unsigned int verbose,
		int debug_channel);

/**   Configure the obstacle avoidance behavior on the helicopter.  
 *   oavoidMode describe the obstacle avoidance behavior as defined in sbconst.h  
 *   SB_OA_xxxx. It can be either:  
 *   - SB_OA_NONE: no obstacle avoidance  
 *   - SB_OA_VERTICAL: only vertical obstacle avoidance (using downward looking  
 *   range sensor)  
 *   - SB_OA_HORIZONTAL: only horizontal obstacle avoidance (using range sensors)  
 *   A OR of the above to combine the behaviors  
 *   */
int sbConfigureOAMode(struct SBControlContext *control, unsigned int oavoidMode);

/**   Configure the communication behavior, in particular how the helicopter state  
 *   is accessed by the controlling computer.  
 *   commMode defined from sbconst.h SB_COM_xxxx. It can be:  
 *   SB_COM_ONREQUEST: the helicopter state is only sent when requested with  
 *   sbRequestState. Arguments frequency and contents are ignored.  
 *   SB_COM_CONTINUOUS: the helicopter will send its state continuously,  
 *   _frequency_ time per second. Only state variable flagged in _contents_ will  
 *   be sent. Maximum frequency achievable on a bluetooth link depends on the  
 *   number of state item requested, and how often the communication direction is  
 *   switched. Values between 10 and 200 seem possible.  
 *   */
int sbConfigureComm(struct SBControlContext *control, 
		unsigned int commMode,
		unsigned int frequency,
		unsigned int numMessages,
		unsigned long contents);

/**   Configure the individual axis control modes. Each axis can be controlled  
 *   either in position (SB_CTRL_POS), velocity (SB_CTRL_VEL) or relative   
 *   displacement (SB_CTRL_REL), or not controlled (SB_CTRL_NONE).   
 *   Not all modes apply to all axis, and some axis are coupled.     
 *   Control Modes defined from sbconst.h SB_CTRL_xxxx  
 *    
 *   Acceptable control are as follows:  
 *   Roll: Position control only  
 *   Pitch: Position control only  
 *   Yaw: Position and Velocity control  
 *   Altitude: Position and Velocity control  */
int sbConfigureControl(struct SBControlContext *control, 
		unsigned int roll,
		unsigned int pitch,
		unsigned int yaw,
		unsigned int altitude);

/**   Configure the timeout policy for the control and general communication  
 *   watchdog. For both timeout, a value of zero does not change the current  
 *   setting. A value of 0xFFFF disactivate the timeout.  
 *   The control timeout is used only when the navigation mode is SB_NAV_CTRLLED.  
 *   In this case, if the controlling application does not issue commands within  
 *   the timeout, the helicopter automatically switch to SB_NAV_HOVER (constant  
 *   height, constant yaw)  
 *   The watchdog timeout is used at any time, but only makes sense in modes  
 *   SB_NAV_CTRLLED and SB_NAV_HOVER. In these mode, if no sbKeepAlive request  
 *   are received within the timeout, the system automatically switch to  
 *   SB_NAV_SINK: constant heading and slowly decreasing altitude until touch  
 *   down. This watchdog is useful to implement a safe behavior in case the  
 *   communication link get broken.  */
int sbConfigureTimeout(struct SBControlContext *control, 
		unsigned short control_timeout_ms,
		unsigned short watchdog_timeout_ms);

/**   Reset the communication watchdog, see sbConfigureTimeout for details.  */
int sbKeepAlive(struct SBControlContext *control); 

/**   Request the helicopter state. This can be used even when communication mode  
 *   is continuous, for instance, to request different data from time to time.  
 *   Given that changing communication direction is very costly with bluetooth,  
 *   it may be better to just augment the number of requested item in continuous  
 *   mode.  */
int sbRequestState(struct SBControlContext *control,
		unsigned long contents, SBHeliState *state);

/**   Receive a state message, as specified in sbConfigureComm.  
 *   _timeout_ms_ represent a timeout (in ms) for the reception of messages. A value of  
 *   zero implies an infinite wait. On success, the state is scaled to  
 *   International System units.  */
int sbReceiveState(struct SBControlContext *control,SBHeliState *state,
		unsigned int timeout_ms);

/**   Wait for data on the communication channel, useful to avoid locking  
 *   the comm channel for too long when receiving state, sbReceiveState  */
int sbWaitState(struct SBControlContext *control, unsigned int timeout_ms);


/**   Set the navigation mode, with navMode defined from sbconst.h SB_NAV_xxxx  
 *   The semantic of navigation mode is as follows:  
 *   SB_NAV_STOP: the helicopter is stopped, on the ground, rotors stopped.  
 *   SB_NAV_IDLE: the helicopter is on the ground, rotors rotating but not enough  
 *     for taking off.  
 *   SB_NAV_TAKEOFF: the helicopter takes off, and transition to SB_NAV_HOVER  
 *     when take off succeeded.  
 *   SB_NAV_HOVER: the helicopter stays at constant height and yaw angle, with no  
 *     pitch or roll. No horizontal stabilisation is active.  
 *     If a watchdog is active, and no KeepAlive request is received after this  
 *     timeout, the helicopter transition to SB_NAV_SINK  
 *   SB_NAV_LAND: the helicopter comes to the ground, and transition to  
 *     SB_NAV_IDLE automatically after touch down.  
 *   SB_NAV_CTRLLED: the helicopter is ready to receive user commands using  
 *     sbSetControl. If a control timeout is active, and no control is received   
 *     after this timeout, then the helicopter goes back to SB_NAV_HOVER  */
int sbSetNavMode(struct SBControlContext *control, 
		unsigned int navmode);

/** Call sbSetNavMode, request a transition to "setmode" and wait for "waitmode"
 * to be reached. Most of the time setmode and waitmode are the same, except
 * for take-off and landif. timeout_sec is the maximum wait time in seconds.
 * State is checked using the sbRequestState function
 * */
int sbSetNavAndWait(struct SBControlContext *control, 
		int setmode, int waitmode, double timout_sec);

/**   Give the set-points of the helicopter control laws. The semantic of the  
 *   set-point depends of the control modes defined with sbConfigureControl.   
 *   Command clipping may be implemented, depending on the helicopter platform.  
 *   If obstacle avoidance is active, it may also take precedence over requested  
 *   control. Control axis where the control mode has been configured to  
 *   SB_CTRL_NONE, can be set to any value (0 is recommended).  
 *   Only SI unit used here: rad, rad/s, m, m/s  */
int sbSetControl(struct SBControlContext *control, 
		double roll, double pitch, double yaw, double altitude);
/** Same as above, but a timestamp can be added to the message
 *   timestamp is in ms, in the same format as the timestamp in the state message, it is ignored by the low-level control
 *   but could be used by an interpolator running on the gumstix and
 *   interpreting the messages for instance. If you don't have such an
 *   interpolator, just set it to zero.
 *   */
int sbSetControlWithTimestamp(struct SBControlContext *control, 
		double roll, double pitch, double yaw, double altitude, unsigned long timestamp);

/**   Configure the speed profile for the raw control mode.  speed profiles can
 *   be specified using the SB_RAWPROFILE_... defines in sbconst.h */
int sbConfigureRawControl(struct SBControlContext *control, 
		unsigned char speedprofile1, unsigned char speedprofile2);

/**   Send raw control command to the motors. The semantic depends on the
 *   platform. This is only accepted in navigation mode SB_NAV_RAW
 *   */
int sbSetRawControl(struct SBControlContext *control, 
		double motor1, double motor2, double servo1, double servo2);


/**   Control the light of the coax speed, if used as a sensor without control 
 *   */
int sbSetCoaxSpeedLight(struct SBControlContext *control, unsigned char percent);

/**   Send a reset command to the DSPIC. After this command, the connections may
 *   not be reusable. 
 *   */
int sbResetDSPIC(struct SBControlContext *control);

/**   Send a string to the DSPIC. If the debug interface are active, then this
 *   string is displayed on the DEBUG console. Otherwise it is ignored.
 *   The string length cannot be longer than SB_STRING_MESSAGE_LENGTH
 *   */
int sbSendString(struct SBControlContext *control,const char text[SB_STRING_MESSAGE_LENGTH]);

/** Get the current value of the trim settings on the helicopter */
int sbGetTrimMode(struct SBControlContext *control, struct SBTrimMode *mode);
/** Set the value of the trim settings on the helicopter */
int sbSetTrimMode(struct SBControlContext *control, const struct SBTrimMode *mode);

/** Get the current value of the control parameters on the helicopter */
int sbGetControlParameters(struct SBControlContext *control, struct SBControlParameters *params);
/** Set the value of the control parameters on the helicopter */
int sbSetControlParameters(struct SBControlContext *control, const struct SBControlParameters *params);

/** Store a custom message on the dsPIC, to be read by the next
 * sbGetCustomMessage call.*/
int sbSetCustomMessage( struct SBControlContext *control, const char message[SB_STRING_MESSAGE_LENGTH]);
/** Read the last custom message from the dsPIC.*/
int sbGetCustomMessage( struct SBControlContext *control, char message[SB_STRING_MESSAGE_LENGTH]);

/** Request version informations about the code running on the target system */
int sbGetVersionInformation(struct SBControlContext *control, 
		SBVersionStatus * version);

/** Configure the bluetooth friendly name and access code */
int sbConfigureBluetooth(struct SBControlContext *control,
		const char code[4], const char name[16]);

/*  ///////////////////////////////////////////////////////////  */
/*    */
/*   Utilities:  */
/*    */
/*   //////////////////////////////////////////////////////////  */

/**   Provide a string representation of constant SBS_xxxx   
 *   defined in sbapi.h. Return NULL for invalid values or combined values  */
const char* sbContentString(unsigned long c);

/** Provide a string representation of constant SB_TRIM_xxxx (defined in
 * sbmessage.c) */
const char* sbTrimModeString(unsigned char mode);

/**   Provide a string representation of constant SB_NAV_xxxx   
 *   defined in sbconst.h. Return NULL for invalid values.  */
const char* sbNavModeString(unsigned char c);

/* Defined in sbstate.h/c, but exported here for wrapper to access */
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

/** Returns current time, as given by gettime ofday, or the correct functions
 * in windows */
double sbGetCurrentTime(void);

#ifdef SBC_HAS_IO
#include <stdio.h>


/** Return the textual description of an error flag. Only works for "pure"
 * error flags, not OR'ed combinations. */
const char * sbFlagString(unsigned char err);

/** Print a description of the error represented by err */
void sbErrorPrint(FILE * fp, unsigned char err);

/**   Print a description of the heli state in fp. Only the part of the state that  
 *   are indicated by state->content are printed  */
void sbStatePrint(FILE * fp, const SBHeliState *state);

/**   Print a textual representation of content in an array form  
 *   e.g: [MODES, TIMESTAMP, RPY, ]  */
void sbContentPrint(FILE *fp, unsigned long content);

/**
 * Print the content of params in the file pointed by fp (could be stdout)
 * The output looks like:
 *
 * */
int sbPrintControlParameters(FILE * fp, const struct SBControlParameters * params);

/**
 * Uses sbPrintControlParameters to save params into fname. Warning, fname is
 * overwritten, not appended
 * */
int sbSaveControlParamters(const char * fname, 
		const struct SBControlParameters * params);

/**
 * Try to read the control paramters in fname.
 * The syntax is the same as in sbPrintControlParameters, so that someone could
 * print the exisiting parameters, modify the file and reload it
 * */
int sbLoadControlParameters(const char * fname,
		struct SBControlParameters * params);
#endif

#ifdef __cplusplus
}
#endif

#endif /*   SB_API_H  */
