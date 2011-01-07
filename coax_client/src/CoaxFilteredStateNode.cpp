/*
 *  File Name:  CoaxFilteredStateNode.cpp
 *  Programmer: Aaron Parker
 *  Date Made:  01-05-2010
 *  Description: ROS node, filters data recieved from coax_server.  Converts 
 */

#include <ros/ros.h>
#include <coax_msgs/CoaxState.h>
#include <coax_client/CoaxFilteredState.h>
#include <CoaxClientConst.h>
#include <math.h>

//global variables
double IR_TUNE[3][2];
double ACCEL_FILTER_K[3];
int PUBLISH_FREQ;
int STATE_MSG_BUFFER;
int MSG_QUEUE;
ros::Publisher filtered_state_pub;

using namespace std;

void stateCallback(const coax_msgs::CoaxStateConstPtr& msg);
double calculateDistance(const double &sensor_value, const double &slope, const double &offset);
void getParams(const ros::NodeHandle &nh);
double roundTwo(const double &num);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coax_filter");
	ros::NodeHandle n("coax_filter");

    getParams(n);
	
	ros::Subscriber state_sub = n.subscribe("/coax_server/state", STATE_MSG_BUFFER, &stateCallback);
	
	filtered_state_pub = n.advertise<coax_client::CoaxFilteredState>("state", MSG_QUEUE);
	
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
    static float prev_accel[3] = {0.0};

    //rounding functions. Rounds rpy values to two decimal places. (getting rid of some error)
	double roll = msg->roll;//roundTwo(msg->roll);
	double pitch = msg->pitch;//roundTwo(msg->pitch);
	double yaw = msg->yaw;//roundTwo(msg->yaw);

	coax_client::CoaxFilteredState new_state = coax_client::CoaxFilteredState();
	
	new_state.header = msg->header;

    for(int i = 0;i<3;i++)
    {
        //convert the distance data from the IR sensors to meter readings.
        new_state.ranges[i] = calculateDistance(max(msg->hranges[i],0.05f),IR_TUNE[i][SLOPE],IR_TUNE[i][OFFSET]);
        
        //low pass filter
	    new_state.accel[i] = ((1.-ACCEL_FILTER_K[i])*prev_accel[i])+(ACCEL_FILTER_K[i]*msg->accel[i]);
        prev_accel[i] = new_state.accel[i];
        //cout << "accel[" << i << "]: " << new_state.accel[i] << endl;
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
        ROS_INFO("Set %s/IR/front_slope to %f",nh.getNamespace().c_str(), IR_TUNE[FRONT][SLOPE]);
    }
    else
    {
        if(nh.hasParam("IR/front_slope"))
            ROS_WARN("%s/IR/front_slope must be a double. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_FRONT_SLOPE);
        else
            ROS_WARN("No value set for %s/IR/front_slope. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_FRONT_SLOPE);
        IR_TUNE[FRONT][SLOPE] = DEFAULT_FRONT_SLOPE;
    }

    if (nh.getParam("IR/left_slope", IR_TUNE[LEFT][SLOPE]))
    {
        ROS_INFO("Set %s/IR/left_slope to %f",nh.getNamespace().c_str(), IR_TUNE[LEFT][SLOPE]);
    }
    else
    {
        if(nh.hasParam("IR/left_slope"))
            ROS_WARN("%s/IR/left_slope must be a double. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_LEFT_SLOPE);
        else
            ROS_WARN("No value set for %s/IR/left_slope. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_LEFT_SLOPE);
        IR_TUNE[LEFT][SLOPE] = DEFAULT_LEFT_SLOPE;
    }

    if (nh.getParam("IR/right_slope", IR_TUNE[RIGHT][SLOPE]))
    {
        ROS_INFO("Set %s/IR/right_slope to %f",nh.getNamespace().c_str(), IR_TUNE[RIGHT][SLOPE]);
    }
    else
    {
        if(nh.hasParam("IR/right_slope"))
            ROS_WARN("%s/IR/right_slope must be a double. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_RIGHT_SLOPE);
        else
            ROS_WARN("No value set for %s/IR/right_slope. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_RIGHT_SLOPE);
        IR_TUNE[RIGHT][SLOPE] = DEFAULT_RIGHT_SLOPE;
    }
    
    //IR params, offset
    if (nh.getParam("IR/front_offset", IR_TUNE[FRONT][OFFSET]))
    {
        ROS_INFO("Set %s/IR/front_offset to %f",nh.getNamespace().c_str(), IR_TUNE[FRONT][OFFSET]);
    }
    else
    {
        if(nh.hasParam("IR/front_offset"))
            ROS_WARN("%s/IR/front_offset must be a double. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_FRONT_OFFSET);
        else
            ROS_WARN("No value set for %s/IR/front_offset. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_FRONT_OFFSET);
        IR_TUNE[FRONT][OFFSET] = DEFAULT_FRONT_OFFSET;
    }

    if (nh.getParam("IR/left_offset", IR_TUNE[LEFT][OFFSET]))
    {
        ROS_INFO("Set %s/IR/left_offset to %f",nh.getNamespace().c_str(), IR_TUNE[LEFT][OFFSET]);
    }
    else
    {
        if(nh.hasParam("IR/left_offset"))
            ROS_WARN("%s/IR/left_offset must be a double. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_LEFT_OFFSET);
        else
            ROS_WARN("No value set for %s/IR/left_offset. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_LEFT_OFFSET);
        IR_TUNE[LEFT][OFFSET] = DEFAULT_LEFT_OFFSET;
    }

    if (nh.getParam("IR/right_offset", IR_TUNE[RIGHT][OFFSET]))
    {
        ROS_INFO("Set %s/IR/right_offset to %f",nh.getNamespace().c_str(), IR_TUNE[RIGHT][OFFSET]);
    }
    else
    {
        if(nh.hasParam("IR/right_offset"))
            ROS_WARN("%s/IR/right_offset must be a double. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_RIGHT_OFFSET);
        else
            ROS_WARN("No value set for %s/IR/right_offset. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_RIGHT_OFFSET);
        IR_TUNE[RIGHT][OFFSET] = DEFAULT_RIGHT_OFFSET;
    }

    //accel k values
    if (nh.getParam("accel/new_x_weight", ACCEL_FILTER_K[X]) && (ACCEL_FILTER_K[X] <= 1 && ACCEL_FILTER_K[X] >= 0))
    {
        ROS_INFO("Set %s/accel/new_x_weight to %f",nh.getNamespace().c_str(), ACCEL_FILTER_K[X]);
    }
    else
    {
        if(nh.hasParam("accel/new_x_weight"))
            ROS_WARN("%s/accel/new_x_weight must be a float between 0 and 1. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_X_FILTER_K);
        else
            ROS_WARN("No value set for %s/accel/new_x_weight. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_X_FILTER_K);
        ACCEL_FILTER_K[X] = DEFAULT_X_FILTER_K;
    }

    if (nh.getParam("accel/new_y_weight", ACCEL_FILTER_K[Y]) && (ACCEL_FILTER_K[Y] <= 1 && ACCEL_FILTER_K[Y] >= 0))
    {
        ROS_INFO("Set %s/accel/new_y_weight to %f",nh.getNamespace().c_str(), ACCEL_FILTER_K[Y]);
    }
    else
    {
        if(nh.hasParam("accel/new_y_weight"))
            ROS_WARN("%s/accel/new_y_weight must be a float between 0 and 1. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_Y_FILTER_K);
        else
            ROS_WARN("No value set for %s/accel/new_y_weight. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_Y_FILTER_K);
        ACCEL_FILTER_K[Y] = DEFAULT_Y_FILTER_K;
    }

    if (nh.getParam("accel/new_z_weight", ACCEL_FILTER_K[Z]) && (ACCEL_FILTER_K[Z] <= 1 && ACCEL_FILTER_K[Z] >= 0))
    {
        ROS_INFO("Set %s/accel/new_z_weight to %f",nh.getNamespace().c_str(), ACCEL_FILTER_K[Z]);
    }
    else
    {
        if(nh.hasParam("accel/new_z_weight"))
            ROS_WARN("%s/accel/new_z_weight must be a double between 0 and 1. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_Z_FILTER_K);
        else
            ROS_WARN("No value set for %s/accel/new__weight. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_Z_FILTER_K);
        ACCEL_FILTER_K[Z] = DEFAULT_Z_FILTER_K;
    }

    //frequency this node publishes a new topic
    if (nh.getParam("publish_freq", PUBLISH_FREQ))
    {
        ROS_INFO("Set %s/publish_freq to %d",nh.getNamespace().c_str(), PUBLISH_FREQ);
    }
    else
    {
        if(nh.hasParam("publish_freq"))
            ROS_WARN("%s/publish_freq must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FSTATE_NODE_PUBLISH_FREQ);
        else
            ROS_WARN("No value set for %s/publish_freq. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FSTATE_NODE_PUBLISH_FREQ);
        PUBLISH_FREQ = DEFAULT_FSTATE_NODE_PUBLISH_FREQ;
    }

    //number of states from coax_server this node will buffer before it begins to drop them
    if (nh.getParam("state_msg_buffer", STATE_MSG_BUFFER))
    {
        ROS_INFO("Set %s/state_msg_buffer to %d",nh.getNamespace().c_str(), STATE_MSG_BUFFER);
    }
    else
    {
        if(nh.hasParam("state_msg_buffer"))
            ROS_WARN("%s/state_msg_buffer must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FSTATE_NODE_STATE_MSG_BUFFER);
        else
            ROS_WARN("No value set for %s/state_msg_buffer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FSTATE_NODE_STATE_MSG_BUFFER);
        STATE_MSG_BUFFER = DEFAULT_FSTATE_NODE_STATE_MSG_BUFFER;
    }

    //number of messages this node will queue for publishing before it drops data
    if (nh.getParam("msg_queue", MSG_QUEUE))
    {
        ROS_INFO("Set %s/msg_queue to %d",nh.getNamespace().c_str(), MSG_QUEUE);
    }
    else
    {
        if(nh.hasParam("msg_queue"))
            ROS_WARN("%s/msg_queue must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FSTATE_NODE_MSG_QUEUE);
        else
            ROS_WARN("No value set for %s/msg_queue. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FSTATE_NODE_MSG_QUEUE);
        MSG_QUEUE = DEFAULT_FSTATE_NODE_MSG_QUEUE;
    }
}

double roundTwo(const double &num)
{
    return floorf(num * 100 +  0.5) / 100;
}
