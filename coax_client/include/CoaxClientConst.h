#ifndef COAX_CONSTANTS_H
#define COAX_CONSTANTS_H

//field of view from COAX webcam, in degrees.
#define DEFAULT_FIELD_OF_VIEW_HORIZ 54.6
#define DEFAULT_FIELD_OF_VIEW_VERT  41.27

//default values used for the blob pattern finder node
#define DEFAULT_BLOB_PATT_NODE_PUBLISH_FREQ 10
#define DEFAULT_BLOB_PATT_NODE_FBLOBS_MSG_BUFFER 10
#define DEFAULT_BLOB_PATT_NODE_MSG_QUEUE 1

//default values used for the blob position tracking node
#define DEFAULT_BLOB_POS_NODE_PUBLISH_FREQ 10
#define DEFAULT_BLOB_POS_NODE_STATE_MSG_BUFFER 1
#define DEFAULT_BLOB_POS_NODE_BLOB_SEQ_MSG_BUFFER 1
#define DEFAULT_BLOB_POS_NODE_MSG_QUEUE 1

//conditional compilation defines
#ifndef FILTER_ACCEL //if 1, filters the accel. readings
#define FILTER_ACCEL            0
#endif
#ifndef CONVERT_ACCEL_TO_GLOBAL //if 1, converts accel. readings to global frame
#define CONVERT_ACCEL_TO_GLOBAL 0
#endif

//default values used to calculate the actual distance detected by IR sensors
#define DEFAULT_FRONT_SLOPE     -41.4835
#define DEFAULT_FRONT_OFFSET    50.7125
#define DEFAULT_LEFT_SLOPE      -39.1334
#define DEFAULT_LEFT_OFFSET     48.1207
#define DEFAULT_RIGHT_SLOPE     -38.8275
#define DEFAULT_RIGHT_OFFSET    47.5877

//used in the low pass filter for the accel data. Weight of new value.
#define DEFAULT_X_FILTER_K  0.5
#define DEFAULT_Y_FILTER_K  0.5
#define DEFAULT_Z_FILTER_K  0.5

//default values used with filtered blobs node
#define DEFAULT_FBLOB_NODE_PUBLISH_FREQ     10
#define DEFAULT_FBLOB_NODE_STATE_MSG_BUFFER 1
#define DEFAULT_FBLOB_NODE_BLOBS_MSG_BUFFER 1
#define DEFAULT_FBLOB_NODE_MSG_QUEUE        1
#define DEFAULT_FBLOB_MIN_BLOB_AREA         300

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

#define BLOB_ADJACENT_THRESH 5
#define BLOB_SEQUENCE_SIZE 4

#define GROUND_ROBOT_ID 18

#endif
