Header header
# Error status set by the helicopter */
uint8 errorFlags
# 
#	 Affected content in this data structure (
#    Use AND with the SBS_... flags above to check the content  
#    e.g: if (state.content & SBS_RPY) {  
#   			compute_odo(state.roll,state.pitch,state.yaw)  
#   		}  
#    This content should correspond to what has been configured in  
#    sbConfigureComm or requested in sbRequestState  *#
uint32 content
# timestamp of the last update, in ms since the initialisation of the
#  helicopter. *#
uint32 timeStamp
# current control timeout (for sending command in SB_NAV_CTRLLED mode) */
uint16 controlTimeout
# current comm timeout, to bring the helicopter back to safety is
#  communication is not maintained. *#
uint16 watchdogTimeout
# Various bit field to represent the system configuration*/
CoaxModes mode

# Current helicopter attitude */
float32 roll
float32 pitch
float32 yaw
# GYRO data */
float32[3] gyro
# Accelerometer data */
float32[3] accel
# Magnetometer data */
float32[3] magneto
# Temperature measured by IMU */
float32 imutemp
# Range measurement in the vertical direction */
float32 zrange
# Filtered altitude, as used by the altitude control in POS mode */
float32 zfiltered
# Output of pressure sensor */
float32 pressure
# Range measurements in the horizontal plane. Sensor placement is
#  platform dependent *#
float32[4] hranges
# Distance to closest obstacle (if implemented) */
float32 xrel
float32 yrel
# Battery voltage */
float32 battery
# Output of the remote control channel, normalised to [-1,1] */
float32[8] rcChannel

CoaxSpeed coaxspeed

# symbols below may be suppressed in future version of the library */

# Output of attitude control (semantic unclear) */
float32[3] o_attitude
# Output of altitude control, i.e. thrust to keep the helicopter affloat32  */
float32 o_altitude
# Output of take-off/landing control (semantic unclear) */
float32 o_tol
# ??? (semantic unclear) */
float32[2] o_xy
# ??? (semantic unclear) */
float32[2] o_oavoid

