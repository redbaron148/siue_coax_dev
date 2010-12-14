#include <ros/ros.h>
#include <coax_msgs/CoaxState.h>
#include <coax_client/CoaxStateFiltered.h>
#include <math.h>

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

//default values used with the node handler
#define DEFAULT_PUBLISH_FREQ        50
#define DEFAULT_STATE_MSG_BUFFER    50
#define DEFAULT_FSTATE_MSG_QUEUE    20

//used for ease of access of accel arrays, accel x axis = accel[X], etc...
enum {X = 0,Y,Z};
enum {FRONT = 0, LEFT, RIGHT};
enum {SLOPE = 0, OFFSET};

//global params, set by getParams function.
double IR_TUNE[3][2];
double ACCEL_FILTER_K[3];
int PUBLISH_FREQ;
int STATE_MSG_BUFFER;
int FSTATE_MSG_QUEUE;

using namespace std;

ros::Publisher filtered_state_pub;

void stateCallback(const coax_msgs::CoaxStateConstPtr& msg);
double calculateDistance(const double &sensor_value, const double &slope, const double &offset);
void getParams(const ros::NodeHandle &nh);
double roundTwo(const double &num);
/*double runningAvg(const double &prev, const double &next, const unsigned int &n)
{
    //ROS_INFO("prev: %f  next: %f  n: %d", prev, next, n);
    return ((prev*(n-1))+next)/double(n);
}*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coax_filtered");
	ros::NodeHandle n("coax_filtered");

    getParams(n);
	
	ros::Subscriber state_sub = n.subscribe("/coax_server/state", STATE_MSG_BUFFER, &stateCallback);
	
	filtered_state_pub = n.advertise<coax_client::CoaxStateFiltered>("state", FSTATE_MSG_QUEUE);
	
	ros::Rate loop_rate(PUBLISH_FREQ);
	
	while(ros::ok())
	{
		ros::spinOnce();
        loop_rate.sleep();
	}
	
	return 0;
}

void stateCallback(const coax_msgs::CoaxStateConstPtr& msg)
{
    //static coax_client::CoaxStateFiltered prev_state;
    static float prev_accel[3] = {0.0};    

    //rounding functions. Rounds rpy values to two decimal places. (getting rid of some error)
	double roll = roundTwo(roll);
	double pitch = roundTwo(pitch);
	double yaw = roundTwo(yaw);

	coax_client::CoaxStateFiltered new_state = coax_client::CoaxStateFiltered();
	
	new_state.header = msg->header;

    for(int i = 0;i<3;i++)
    {
        //convert the distance data from the IR sensors to meter readings.
        new_state.ranges[0] = calculateDistance(max(msg->hranges[i],0.05f),IR_TUNE[i][SLOPE],IR_TUNE[i][OFFSET]);
        
        //low pass filter
	    new_state.accel[i] = (1.-ACCEL_FILTER_K[i])*prev_accel[i]+ACCEL_FILTER_K[i]*msg->accel[i];
        prev_accel[i] = new_state.accel[i];
    }

    //transform local acceleration values to global values.  accel z should generally stay -9.8 (gravity)
    new_state.global_accel[X] = cos(pitch)*cos(yaw)*new_state.accel[X]-sin(yaw)*cos(pitch)*new_state.accel[Y]+sin(pitch)*new_state.accel[Z];
    new_state.global_accel[Y] = ((cos(yaw)*sin(roll)*sin(pitch)+cos(roll)*sin(yaw))*new_state.accel[X])-
		      ( ( ( sin(yaw)*sin(roll)*sin(pitch) ) - ( cos(roll)*cos(yaw) ) ) *new_state.accel[Y])-
		      (sin(roll)*cos(pitch)*new_state.accel[Z]);
    new_state.global_accel[Z] = (((-sin(pitch)*cos(roll)*cos(yaw))+(sin(roll)*sin(yaw)))*new_state.accel[X])+
		      (((sin(pitch)*sin(yaw)*cos(roll))+(sin(roll)*cos(yaw)))*new_state.accel[Y])+
		      (cos(roll)*cos(pitch)*new_state.accel[Z]);

	filtered_state_pub.publish(new_state);
}

double calculateDistance(const double &sensor_value, const double &slope, const double &offset)
{
	return (slope*log(sensor_value)+offset);
}

void getParams(const ros::NodeHandle &nh)
{
    //IR params, slope
    if (nh.getParam("IR/front_slope", IR_TUNE[FRONT][SLOPE]))
    {
        ROS_INFO("Set IR/front_slope to %f.",IR_TUNE[FRONT][SLOPE]);
    }
    else
    {
        if(nh.hasParam("IR/front_slope"))
            ROS_WARN("IR/front_slope must be a double. Setting default value.");
        else
            ROS_WARN("No value set for IR/front_slope. Setting default value.");
        IR_TUNE[FRONT][SLOPE] = DEFAULT_FRONT_SLOPE;
    }

    if (nh.getParam("IR/left_slope", IR_TUNE[LEFT][SLOPE]))
    {
        ROS_INFO("Set IR/left_slope to %f.",IR_TUNE[LEFT][SLOPE]);
    }
    else
    {
        if(nh.hasParam("IR/left_slope"))
            ROS_WARN("IR/left_slope must be a double. Setting default value.");
        else
            ROS_WARN("No value set for IR/left_slope. Setting default value.");
        IR_TUNE[LEFT][SLOPE] = DEFAULT_LEFT_SLOPE;
    }

    if (nh.getParam("IR/right_slope", IR_TUNE[RIGHT][SLOPE]))
    {
        ROS_INFO("Set IR/right_slope to %f.",IR_TUNE[RIGHT][SLOPE]);
    }
    else
    {
        if(nh.hasParam("IR/right_slope"))
            ROS_WARN("IR/right_slope must be a double. Setting default value.");
        else
            ROS_WARN("No value set for IR/right_slope. Setting default value.");
        IR_TUNE[RIGHT][SLOPE] = DEFAULT_RIGHT_SLOPE;
    }
    
    //IR params, offset
    if (nh.getParam("IR/front_offset", IR_TUNE[FRONT][OFFSET]))
    {
        ROS_INFO("Set IR/front_offset to %f.",IR_TUNE[FRONT][OFFSET]);
    }
    else
    {
        if(nh.hasParam("IR/front_offset"))
            ROS_WARN("IR/front_offset must be a double. Setting default value.");
        else
            ROS_WARN("No value set for IR/front_offset. Setting default value.");
        IR_TUNE[FRONT][OFFSET] = DEFAULT_FRONT_OFFSET;
    }

    if (nh.getParam("IR/left_offset", IR_TUNE[LEFT][OFFSET]))
    {
        ROS_INFO("Set IR/left_offset to %f.",IR_TUNE[LEFT][OFFSET]);
    }
    else
    {
        if(nh.hasParam("IR/left_offset"))
            ROS_WARN("IR/left_offset must be a double. Setting default value.");
        else
            ROS_WARN("No value set for IR/left_offset. Setting default value.");
        IR_TUNE[LEFT][OFFSET] = DEFAULT_LEFT_OFFSET;
    }

    if (nh.getParam("IR/right_offset", IR_TUNE[RIGHT][OFFSET]))
    {
        ROS_INFO("Set IR/right_offset to %f.",IR_TUNE[RIGHT][OFFSET]);
    }
    else
    {
        if(nh.hasParam("IR/right_offset"))
            ROS_WARN("IR/right_offset must be a double. Setting default value.");
        else
            ROS_WARN("No value set for IR/right_offset. Setting default value.");
        IR_TUNE[RIGHT][OFFSET] = DEFAULT_RIGHT_OFFSET;
    }

    //accel k values
    if (nh.getParam("accel/new_x_weight", ACCEL_FILTER_K[X]) && (ACCEL_FILTER_K[X] <= 1 && ACCEL_FILTER_K[X] >= 0))
    {
        ROS_INFO("Set accel/new_x_weight to %f.",ACCEL_FILTER_K[X]);
    }
    else
    {
        if(nh.hasParam("accel/new_x_weight"))
            ROS_WARN("accel/new_x_weight must be a double between 0 and 1. Setting default value.");
        else
            ROS_WARN("No value set for accel/new_x_weight. Setting default value.");
        ACCEL_FILTER_K[X] = DEFAULT_X_FILTER_K;
    }

    if (nh.getParam("accel/new_y_weight", ACCEL_FILTER_K[Y]) && (ACCEL_FILTER_K[Y] <= 1 && ACCEL_FILTER_K[Y] >= 0))
    {
        ROS_INFO("Set accel/new_y_weight to %f.",ACCEL_FILTER_K[Y]);
    }
    else
    {
        if(nh.hasParam("accel/new_y_weight"))
            ROS_WARN("accel/new_y_weight must be a double between 0 and 1. Setting default value.");
        else
            ROS_WARN("No value set for accel/new_y_weight. Setting default value.");
        ACCEL_FILTER_K[Y] = DEFAULT_Y_FILTER_K;
    }

    if (nh.getParam("accel/new_z_weight", ACCEL_FILTER_K[Z]) && (ACCEL_FILTER_K[Z] <= 1 && ACCEL_FILTER_K[Z] >= 0))
    {
        ROS_INFO("Set accel/new_z_weight to %f.",ACCEL_FILTER_K[Z]);
    }
    else
    {
        if(nh.hasParam("accel/new_z_weight"))
            ROS_WARN("accel/new_z_weight must be a double between 0 and 1. Setting default value.");
        else
            ROS_WARN("No value set for accel/new__weight. Setting default value.");
        ACCEL_FILTER_K[Z] = DEFAULT_Z_FILTER_K;
    }

    /*//number of messages between starting and ending
    if (nh.getParam("avg_period", AVG_PERIOD))
    {
        ROS_INFO("Set avg_period to %d.", AVG_PERIOD);
    }
    else
    {
        if(nh.hasParam("avg_period"))
            ROS_WARN("avg_period must be an integer. Setting default value.");
        else
            ROS_WARN("No value set for avg_period. Setting default value.");
        AVG_PERIOD = DEFAULT_AVG_PERIOD;
    }*/

    //frequency this node publishes a new topic
    if (nh.getParam("publish_freq", PUBLISH_FREQ))
    {
        ROS_INFO("Set publish_freq to %d.", PUBLISH_FREQ);
    }
    else
    {
        if(nh.hasParam("publish_freq"))
            ROS_WARN("publish_freq must be an integer. Setting default value.");
        else
            ROS_WARN("No value set for publish_freq. Setting default value.");
        PUBLISH_FREQ = DEFAULT_PUBLISH_FREQ;
    }

    //number of states from coax_server this node will buffer before it begins to drop them
    if (nh.getParam("state_msg_buffer", STATE_MSG_BUFFER))
    {
        ROS_INFO("Set state_msg_buffer to %d.", STATE_MSG_BUFFER);
    }
    else
    {
        if(nh.hasParam("state_msg_buffer"))
            ROS_WARN("state_msg_buffer must be an integer. Setting default value.");
        else
            ROS_WARN("No value set for state_msg_buffer. Setting default value.");
        STATE_MSG_BUFFER = DEFAULT_STATE_MSG_BUFFER;
    }

    //number of messages this node will queue for publishing before it drops data
    if (nh.getParam("fstate_msg_queue", FSTATE_MSG_QUEUE))
    {
        ROS_INFO("Set fstate_msg_queue to %d.", FSTATE_MSG_QUEUE);
    }
    else
    {
        if(nh.hasParam("fstate_msg_queue"))
            ROS_WARN("fstate_msg_queue must be an integer. Setting default value.");
        else
            ROS_WARN("No value set for fstate_msg_queue. Setting default value.");
        FSTATE_MSG_QUEUE = DEFAULT_FSTATE_MSG_QUEUE;
    }
}

double roundTwo(const double &num)
{
    return floorf(num * 1000 +  0.5) / 1000;
}
