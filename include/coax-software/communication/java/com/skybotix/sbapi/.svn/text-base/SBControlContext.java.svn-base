package com.skybotix.sbapi;

import java.io.IOException;
import java.io.OutputStream;
import java.util.UUID;


/** 
 * \file sbapi.h Data selection flags for skybotics platform communication
 * This is not defined in sbconst.h to keep the header 16bit only
 *  General remarks: All the functions below return an int. The
 * semantic is (as usual in C programming) 0 for no error. Any other number
 * corresponds to an error situation. TODO: document error codes
 * */



import com.skybotix.sbapi.SBHeliState;
import com.skybotix.sbapi.SBChannel;
import com.skybotix.sbapi.SBVersionStatus;
import com.skybotix.sbapi.SBMessages;

public class SBControlContext {
    public SBChannel channel = null;
    private int ackMode = 1;
    private SBStateCallback callback = null;

    /**   Open a network communication channel to the helicopter.     This may be
     * used in simulation or possibly to communicate via an on-board
     * network-enabled processor.   */
    public int sbOpenSocketCommunication(String hostname, int port) {
        return -1;
    }

    public int sbOpenAndroidBTCommunication(String macaddress) {
        return -1;
    }

    /**   Flush the communication channel   */
    public int sbFlushCommunication() {
        return channel.flush();
    }

    /**   Close the communication channel and release any resource if needed  */
    public int sbCloseCommunication() {
        return channel.close();
    }

    /**   Lock the communication channel to prevent concurrent access from
     * different     threads. Currently, this will only work on POSIX systems.  */
    public int sbLockCommunication() {
        return channel.lock();
    }

    /**   Unlock the communication channel  */
    public int sbUnlockCommunication() {
        return channel.unlock();
    }

    /** Open a connection to the helicopter. This is required in order to get the
     * permissions to change parameters or control the helicopter. For receiving
     * data, there is no need to connect  */
    public int sbConnect() {
        return -1;
    }

    /** Release the connection to the helicopter */
    public int sbDisconnect() {
        return -1;
    }

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

    public class SBStateCallback {
        void call(SBHeliState state) {
        }
    }
    public int sbRegisterStateCallback(SBStateCallback _callback) {
        callback = _callback;
        return 0;
    }

    /**   Returns a content mask, where *list is a OR of the SBS_xxxx flags which
     * make     sense on this platform.  */
    public int sbGetSensorList(long list) {
        list = 0;
        return -1;
    }

    /**   Configure the communication so that control requests (sbSetNavMode,
     * sbKeepAlive, sbSetControl) requires an acknowledgement     from the
     * helicopter. If requestAck is zero, all control functions      will not
     * confirm reception of their commands. With bluetooth communication,     this
     * can improve the communication rate. If requestAck is non zero, the
     * control request will be followed by an acknowledgement.  
     * */
    public int sbConfigureAckMode( int requestAck) {
        return -1;
    }

    /**   Set commloop verbosity, for debug mode  
     *   */
    public int sbConfigureCommLoop( int verbose, int debug_channel) {
        return -1;
    }

    /**   Configure the obstacle avoidance behavior on the helicopter.  
     *   oavoidMode describe the obstacle avoidance behavior as defined in sbconst.h  
     *   SB_OA_xxxx. It can be either:  
     *   - SB_OA_NONE: no obstacle avoidance  
     *   - SB_OA_VERTICAL: only vertical obstacle avoidance (using downward looking  
     *   range sensor)  
     *   - SB_OA_HORIZONTAL: only horizontal obstacle avoidance (using range sensors)  
     *   A OR of the above to combine the behaviors  
     *   */
    public int sbConfigureOAMode( int oavoidMode) {
        return -1;
    }

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
    public int sbConfigureComm( int commMode, int frequency,
            int numMessages, long contents) {
        return -1;
    }

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
    public int sbConfigureControl( int roll, int pitch,
            int yaw, int altitude) {
        return -1;
    }

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
    public int sbConfigureTimeout( short control_timeout_ms,
            short watchdog_timeout_ms) {
        return -1;
    }

    /**   Reset the communication watchdog, see sbConfigureTimeout for details.  */
    public int sbKeepAlive() {
        return -1;
    }

    /**   Request the helicopter state. This can be used even when communication mode  
     *   is continuous, for instance, to request different data from time to time.  
     *   Given that changing communication direction is very costly with bluetooth,  
     *   it may be better to just augment the number of requested item in continuous  
     *   mode.  */
    public int sbRequestState( long contents, SBHeliState state) {
        return -1;
    }

    /**   Receive a state message, as specified in sbConfigureComm.  
     *   _timeout_ms_ represent a timeout (in ms) for the reception of messages. A value of  
     *   zero implies an infinite wait. On success, the state is scaled to  
     *   International System units.  */
    public int sbReceiveState(SBHeliState state, int timeout_ms) {
        return -1;
    }

    /**   Wait for data on the communication channel, useful to avoid locking  
     *   the comm channel for too long when receiving state, sbReceiveState  */
    public int sbWaitState( int timeout_ms) {
        return -1;
    }


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
    public int sbSetNavMode( int navmode) {
        return -1;
    }

    /** Call sbSetNavMode, request a transition to "setmode" and wait for "waitmode"
     * to be reached. Most of the time setmode and waitmode are the same, except
     * for take-off and landif. timeout_sec is the maximum wait time in seconds.
     * State is checked using the sbRequestState function
     * */
    public int sbSetNavAndWait( int setmode, int waitmode, double timout_sec) {
        return -1;
    }

    /**   Give the set-points of the helicopter control laws. The semantic of the  
     *   set-point depends of the control modes defined with sbConfigureControl.   
     *   Command clipping may be implemented, depending on the helicopter platform.  
     *   If obstacle avoidance is active, it may also take precedence over requested  
     *   control. Control axis where the control mode has been configured to  
     *   SB_CTRL_NONE, can be set to any value (0 is recommended).  
     *   Only SI unit used here: rad, rad/s, m, m/s  */
    public int sbSetControl( double roll, double pitch, double yaw, double altitude) {
        return -1;
    }
    /** Same as above, but a timestamp can be added to the message
     *   timestamp is in ms, in the same format as the timestamp in the state message, it is ignored by the low-level control
     *   but could be used by an interpolator running on the gumstix and
     *   interpreting the messages for instance. If you don't have such an
     *   interpolator, just set it to zero.
     *   */
    public int sbSetControlWithTimestamp( 
            double roll, double pitch, double yaw, double altitude, 
            long timestamp) {
        return -1;
            }

    /**   Configure the speed profile for the raw control mode.  speed profiles can
     *   be specified using the SB_RAWPROFILE_... defines in sbconst.h */
    public int sbConfigureRawControl( char speedprofile1, char speedprofile2) {
        return -1;
    }

    /**   Send raw control command to the motors. The semantic depends on the
     *   platform. This is only accepted in navigation mode SB_NAV_RAW
     *   */
    public int sbSetRawControl( double motor1, double motor2, 
            double servo1, double servo2) {
        return -1;
    }


    /**   Control the light of the coax speed, if used as a sensor without control 
     *   */
    public int sbSetCoaxSpeedLight( char percent) {
        return -1;
    }

    /**   Send a reset command to the DSPIC. After this command, the connections may
     *   not be reusable. 
     *   */
    public int sbResetDSPIC() {
        return -1;
    }

    /**   Send a string to the DSPIC. If the debug interface are active, then this
     *   string is displayed on the DEBUG console. Otherwise it is ignored.
     *   The string length cannot be longer than SB_STRING_MESSAGE_LENGTH
     *   */
    public int sbSendString(byte[] message) {
        return -1;
    }

    /** \struct representing the trim settings  */
    public class SBTrimMode {
        /** Trim mode, can be SB_TRIM_FROM_RC or SB_TRIM_SOFTWARE */
        public byte trimMode; 
        /** Trim position for the roll axis */
        public float rollTrim;
        /** Trim position for the pitch axis */
        public float pitchTrim;
    }

    /** Get the current value of the trim settings on the helicopter */
    public int sbGetTrimMode(SBTrimMode mode) {
        return -1;
    }
    /** Set the value of the trim settings on the helicopter */
    public int sbSetTrimMode( SBTrimMode mode) {
        return -1;
    }

    /** \struct representing the control parameters  */
    public class SBControlParameters {
        /** Base thrust around which the control operates */
        public float baseThrust;
        /** Difference of operating point between up and down motors */
        public float yawOffset;
        /** Altitude gain: Kp*/
        public float altitudeKp;
        /** Altitude gain: Ki*/
        public float altitudeKi;
        /** Altitude gain: Kd*/
        public float altitudeKd;

        /** Yaw gain: Kp*/
        public float yawKp;
        /** Yaw gain: Ki*/
        public float yawKi;
        /** Yaw gain: Kd*/
        public float yawKd;
    }

    /** Get the current value of the control parameters on the helicopter */
    public int sbGetControlParameters( SBControlParameters params) {
        return -1;
    }
    /** Set the value of the control parameters on the helicopter */
    public int sbSetControlParameters( SBControlParameters params) {
        return -1;
    }

    /** Store a custom message on the dsPIC, to be read by the next
     * sbGetCustomMessage call, only te first SB_STRING_MESSAGE_LENGTH are used.*/
    public int sbSetCustomMessage(  byte[] message) {
        return -1;
    }
    /** Read the last custom message from the dsPIC.*/
    public int sbGetCustomMessage(  byte[] message) {
        return -1;
    }

    /** Request version informations about the code running on the target system */
    public int sbGetVersionInformation( SBVersionStatus version) {
        return -1;
    }

    /** Configure the bluetooth friendly name and access code */
    public int sbConfigureBluetooth( byte[] code /* 4 bytes */, 
            byte[] name /* 16 bytes at most */) {
        return -1;
    }

    /*  ///////////////////////////////////////////////////////////  */
    /*    */
    /*   Utilities:  */
    /*    */
    /*   //////////////////////////////////////////////////////////  */

    /**   Provide a string representation of constant SBS_xxxx   
     *   defined in sbapi.h. Return NULL for invalid values or combined values  */
    public static String  sbContentString(long c) {
        return null;
    }

    /** Provide a string representation of constant SB_TRIM_xxxx (defined in
     * sbmessage.c) */
    public static String  sbTrimModeString(byte mode) {
        return null;
    }

    /* Defined in sbstate.h/c, but exported here for wrapper to access */
    /**   Provide a string representation of constant SB_CTRL_xxxx   
     *   defined in sbconst.h. Return NULL for invalid values.  */
    public static String  sbCtrlModeString(byte c) {
        return null;
    }
    /**   Provide a string representation of constant SB_COM_xxxx   
     *   defined in sbconst.h. Return NULL for invalid values.  */
    public static String  sbCommModeString(byte c) {
        return null;
    }
    /**   Provide a string representation of constant SB_OA_xxxx   
     *   defined in sbconst.h. Return NULL for invalid values.  */
    public static String  sbOAModeString(byte c) {
        return null;
    }
    /**   Provide a string representation of constant SB_NAV_xxxx   
     *   defined in sbconst.h. Return NULL for invalid values.  */
    public static String  sbNavModeString(byte c) {
        return null;
    }

    /** Returns current time, as given by gettime ofday, or the correct functions
     * in windows */
    public static double sbGetCurrentTime() {
        return 0.0;
    }

    /** Return the textual description of an error flag. Only works for "pure"
     * error flags, not OR'ed combinations. */
    public static String  sbFlagString(byte err) {
        return null;
    }

    /** Print a description of the error represented by err */
    public static void sbErrorPrint(OutputStream out, byte err) {
    }

    /**   Print a description of the heli state in fp. Only the part of the state that  
     *   are indicated by state->content are printed  */
    public static void sbStatePrint(OutputStream out, SBHeliState state) {
    }

    /**   Print a textual representation of content in an array form  
     *   e.g: [MODES, TIMESTAMP, RPY, ]  */
    public static void sbContentPrint(OutputStream out, long content) {
    }

    /**
     * Print the content of params in the file pointed by fp (could be stdout)
     * The output looks like:
     *
     * */
    public static void sbPrintControlParameters(OutputStream out, SBControlParameters params) {
    }

    /**
     * Uses sbPrintControlParameters to save params into fname. Warning, fname is
     * overwritten, not appended
     * */
    public int sbSaveControlParamters(String  fname, 
            SBControlParameters params) {
        return -1;
    }

    /**
     * Try to read the control paramters in fname.
     * The syntax is the same as in sbPrintControlParameters, so that someone could
     * print the exisiting parameters, modify the file and reload it
     * */
    public int sbLoadControlParameters(String  fname,
            SBControlParameters params) {
        return -1;
    }
}

