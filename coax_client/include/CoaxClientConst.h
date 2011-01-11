#ifndef COAX_CONSTANTS
#define COAX_CONSTANTS

//default values used to calculate the actual distance detected by IR sensors
#define DEFAULT_FRONT_SLOPE         -41.4835
#define DEFAULT_FRONT_OFFSET        50.7125
#define DEFAULT_LEFT_SLOPE          -39.1334
#define DEFAULT_LEFT_OFFSET         48.1207
#define DEFAULT_RIGHT_SLOPE         -38.8275
#define DEFAULT_RIGHT_OFFSET        47.5877

//used in the low pass filter for the accel data. Weight of new value.
#define DEFAULT_X_FILTER_K          0.5
#define DEFAULT_Y_FILTER_K          0.5
#define DEFAULT_Z_FILTER_K          0.5

//default values used with filtered state node
#define DEFAULT_FSTATE_NODE_PUBLISH_FREQ        50
#define DEFAULT_FSTATE_NODE_STATE_MSG_BUFFER    50
#define DEFAULT_FSTATE_NODE_MSG_QUEUE           20

//default values used with localization node
#define DEFAULT_LOC_NODE_PUBLISH_FREQ        50
#define DEFAULT_LOC_NODE_FSTATE_MSG_BUFFER   50
#define DEFAULT_LOC_NODE_MSG_QUEUE           20
#define DEFAULT_LOC_NODE_UPDATE_PERIOD       10

//used for ease of access of accel arrays, accel x axis = accel[X], etc...
#define X       0
#define Y       1
#define Z       2

//sensor port number, front IR is 0, left IR is 1, etc...
#define FRONT   0
#define LEFT    1
#define RIGHT   2

//used for calculating human readable distance data from IR sensors.
//slope 
#define SLOPE   0
#define OFFSET  1

#endif
