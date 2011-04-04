
#No obstacle avoidance 
uint8 SB_OA_NONE=0
#Horizontal obstacle avoidance 
uint8 SB_OA_HORIZONTAL=1
#Vertical obstacle avoidance 
uint8 SB_OA_VERTICAL=2

#Communication on request only 
uint8 SB_COM_ONREQUEST=0
#Continuous communication 
uint8 SB_COM_CONTINUOUS=1

#No control 
uint16 SB_CTRL_NONE=0
#Position control 
uint16 SB_CTRL_POS=1
#Relative set point control (wrt range for instance) 
uint16 SB_CTRL_REL=2
#Velocity control 
uint16 SB_CTRL_VEL=3
#Acceleration/Force control, currently only used for thrust 
uint16 SB_CTRL_FORCE=4
#Manual control 
uint16 SB_CTRL_MANUAL=8

#Step velocity profile for servos in raw mode 
uint8 SB_RAWPROFILE_STEP=0
#Ramp velocity profile for servos in raw mode 
uint8 SB_RAWPROFILE_RAMP=1

#Navigation mode: STOP 
uint8 SB_NAV_STOP=0
#Navigation mode: IDLE 
uint8 SB_NAV_IDLE=1
#Navigation mode: TAKEOFF 
uint8 SB_NAV_TAKEOFF=2
#Navigation mode: LAND 
uint8 SB_NAV_LAND=3
#Navigation mode: HOVER 
uint8 SB_NAV_HOVER=4
#Navigation mode: CTRLLED 
uint8 SB_NAV_CTRLLED=5
#Navigation mode: SINK 
uint8 SB_NAV_SINK=6
#Navigation mode: RAW 
uint8 SB_NAV_RAW=7
#Navigation mode: MANUAL (unused) 
uint8 SB_NAV_MANUAL=8

#Error code in acknowledgement message: no error 
uint8 SB_REPLY_OK=   00
#Error code in acknowledgement message: invalid navigation mode 
uint8 SB_REPLY_INVALID_NAVMODE=253
#Error code in acknowledgement message: unknown message id or request 
uint8 SB_REPLY_UNKNOWN=254
#Error code in acknowledgement message: some error 
uint8 SB_REPLY_ERROR=255
#Error code in acknowledgement message: could not decode message 
uint8 SB_REPLY_DECODE_FAILURE=252
#Error code in acknowledgement message: request not accepted because system
#  is busy with another connection
uint8 SB_REPLY_BUSY=251
#Error code in acknowledgement message: too early to do this request 
uint8 SB_REPLY_TOO_EARLY=250

#Identification of the range measurement 
uint32 SB_RANGE_FRONT=0
uint32 SB_RANGE_LEFT=1
uint32 SB_RANGE_RIGHT=2
uint32 SB_RANGE_BACK=3

#Id of trim modes 
uint32 SB_TRIM_FROM_RC=0
uint32 SB_TRIM_SOFTWARE=1

#If for RC channels 
uint32 SB_RC_THROTTLE=0
uint32 SB_RC_THROTTLE_TRIM=1
uint32 SB_RC_YAW=2
uint32 SB_RC_YAW_TRIM=3
uint32 SB_RC_ROLL=4
uint32 SB_RC_ROLL_TRIM=5
uint32 SB_RC_PITCH=6
uint32 SB_RC_PITCH_TRIM=7

int32 DEBUG_CHANNEL_NONE=-1
int32 DEBUG_CHANNEL_GUMSTIX=0
int32 DEBUG_CHANNEL_BLUETOOTH=1

#System mode (nav, comm, ...) 
uint32 SBS_MODES=1
#System timestamp 
uint32 SBS_TIMESTAMP=2
#Roll, pitch, yaw from IMU 
uint32 SBS_RPY=4
#Roll, pitch, yaw rate from IMU 
uint32 SBS_GYRO=8
#Acceleration from IMU 
uint32 SBS_ACCEL=16
#Magnetic field vector from IMU 
uint32 SBS_MAGNETO=32
#Temperature from IMU 
uint32 SBS_IMUTEMP=64
#Range to the floor, and filtered altitude 
uint32 SBS_ALTITUDE=128
#Altitude from pressure sensor 
uint32 SBS_PRESSURE=256
#Horizontal ranges to obstacles 
uint32 SBS_HRANGES=512
#Distance to closest object 
uint32 SBS_XY_REL=1024
#Battery status 
uint32 SBS_BATTERY=2048
#Timeout currently used 
uint32 SBS_TIMEOUT=4096
#Output of Attitude control 
uint32 SBS_O_ATTITUDE=8192
#Output of Altitude control 
uint32 SBS_O_ALTITUDE=16384
#Output of TakeOff/Landing control 
uint32 SBS_O_TOL=32768
#Output of XY control 
uint32 SBS_O_XY=65536
#Output of Obstacle avoidance control 
uint32 SBS_O_OAVOID=131072

#Output of Obstacle avoidance control 
uint32 SBS_CHANNELS=262144

#Combined flags for convenience 
uint32 SBS_ALL=16777215
uint32 SBS_IMU_ALL=124
uint32 SBS_RANGES_ALL=640
uint32 SBS_ALTITUDE_ALL=384
uint32 SBS_OUTPUT_ALL=253952


