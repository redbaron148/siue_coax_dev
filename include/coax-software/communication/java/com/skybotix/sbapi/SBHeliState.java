package com.skybotix.sbapi;


public class SBHeliState  {
    /** Error status set by the helicopter */
    public byte errorFlags;

    /** System mode (nav, comm, ...) */
    final public long SBS_MODES =	 0x00000001	;
    /** System timestamp */
    final public long SBS_TIMESTAMP = 0x00000002	;
    /** Roll, pitch, yaw from IMU */
    final public long SBS_RPY =		 0x00000004	;
    /** Roll, pitch, yaw rate from IMU */
    final public long SBS_GYRO =	 0x00000008	;
    /** Acceleration from IMU */
    final public long SBS_ACCEL =	 0x00000010	;
    /** Magnetic field vector from IMU */
    final public long SBS_MAGNETO =	 0x00000020	;
    /** Temperature from IMU */
    final public long SBS_IMUTEMP =	 0x00000040	;
    /** Range to the floor, and filtered altitude */
    final public long SBS_ALTITUDE = 0x00000080	;
    /** Altitude from pressure sensor */
    final public long SBS_PRESSURE = 0x00000100	;
    /** Horizontal ranges to obstacles */
    final public long SBS_HRANGES =	 0x00000200	;
    /** Distance to closest object */
    final public long SBS_XY_REL =	 0x00000400	;
    /** Battery status */
    final public long SBS_BATTERY =	 0x00000800	;
    /** Timeout currently used */
    final public long SBS_TIMEOUT =	 0x00001000	;
    /** Output of Attitude control */
    final public long SBS_O_ATTITUDE = 0x00002000	;
    /** Output of Altitude control */
    final public long SBS_O_ALTITUDE = 0x00004000	;
    /** Output of TakeOff/Landing control */
    final public long SBS_O_TOL =	 0x00008000	;
    /** Output of XY control */
    final public long SBS_O_XY =	 0x00010000	;
    /** Output of Obstacle avoidance control */
    final public long SBS_O_OAVOID = 0x00020000	;

    /** Remote control =channels */;
    final public long SBS_CHANNELS = 0x00040000	;

    /** Coax speed =status */;
    final public long SBS_COAXSPEED =	 0x00080000	;

    /** Combined flags =for convenience */;
    final public long SBS_ALL =		 0x000FFFFF;
    final public long SBS_IMU_ALL =	 0x0000007C;
    final public long SBS_RANGES_ALL = 0x00000280;
    final public long SBS_ALTITUDE_ALL =0x00000180;
    final public long SBS_OUTPUT_ALL = 0x0003E000;


    /** 
     *	 Affected content in this data structure (
     *   Use AND with the SBS_... flags above to check the content  
     *   e.g: if (state.content & SBS_RPY) {  
     *  			compute_odo(state.roll,state.pitch,state.yaw);  
     *  		}  
     *   This content should correspond to what has been configured in  
     *   sbConfigureComm or requested in sbRequestState  */
    public long content;
    /** timestamp of the last update, in ms since the initialisation of the
     * helicopter. */
    public long timeStamp;
    /** current control timeout (for sending command in SB_NAV_CTRLLED mode) */
    public short controlTimeout;
    /** current comm timeout, to bring the helicopter back to safety is
     * communication is not maintained. */
    public short watchdogTimeout;
    /** Various bit field to represent the system configuration*/
    public class SBNavMode {
        /** Navigation mode: SB_NAV_... */
        public byte navigation;
        /** Communication mode: SB_COM_... */
        public byte communication;
        /** Obstacle avoidance mode: or of SB_OA_... */
        public byte oavoid;
        /** Control mode for roll axis: SB_CTRL_... */
        public byte rollAxis;
        /** Control mode for pitch axis: SB_CTRL_... */
        public byte pitchAxis;
        /** Control mode for yaw axis: SB_CTRL_... */
        public byte yawAxis;
        /** Control mode for altitude axis: SB_CTRL_... */
        public byte altAxis;
    };
    public SBNavMode mode;

    /** Current helicopter attitude */
    public float roll, pitch, yaw;
    /** GYRO data */
    public float[] gyro = new float[3];
    /** Accelerometer data */
    public float[] accel = new float[3];
    /** Magnetometer data */
    public float[] magneto = new float[3];
    /** Temperature measured by IMU */
    public float imutemp;
    /** Range measurement in the vertical direction */
    public float zrange;
    /** Filtered altitude, as used by the altitude control in POS mode */
    public float zfiltered;
    /** Output of pressure sensor */
    public float pressure;
    /** Range measurements in the horizontal plane. Sensor placement is
     * platform dependent */
    public float[] hranges = new float[4];
    /** Distance to closest obstacle (if implemented) */
    public float xrel, yrel;
    /** Battery voltage */
    public float battery;
    /** Output of the remote control channel, normalised to [-1,1] */
    public float[] rcChannel = new float[8];

    public class SBCoaxSpeed {
        public byte state; /** see sbconst.h COAXSPEED_* constants */
        public byte light;
        public float vel_x, vel_y;
    };
    public SBCoaxSpeed coaxspeed;

    /* symbols below may be suppressed in future version of the library */

    /** Output of attitude control (semantic unclear) */
    public float[] o_attitude = new float[3];
    /** Output of altitude control, i.e. thrust to keep the helicopter affloat  */
    public float o_altitude;
    /** Output of take-off/landing control (semantic unclear) */
    public short o_tol;
    /** ??? (semantic unclear) */
    public float[] o_xy = new float[2];
    /** ??? (semantic unclear) */
    public float[] o_oavoid = new float[2];
} 
