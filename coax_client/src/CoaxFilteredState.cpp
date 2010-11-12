#include "ros/ros.h"
#include "coax_msgs/CoaxState.h"
#include "coax_client/CoaxStateFiltered.h"
#include <math.h>

#define DEFAULT_FRONT_SLOPE  -41.4835
#define DEFAULT_FRONT_OFFSET  50.7125
#define DEFAULT_LEFT_SLOPE   -39.1334
#define DEFAULT_LEFT_OFFSET   48.1207
#define DEFAULT_RIGHT_SLOPE  -38.8275
#define DEFAULT_RIGHT_OFFSET  47.5877

using namespace std;

ros::Publisher filtered_state_pub;

void stateCallback(const coax_msgs::CoaxStateConstPtr& msg);
double calculateDistance(const double &sensor_value, const double &slope, const double &offset);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coax_filtered");
	ros::NodeHandle n("/coax_filtered");
	
	ros::Subscriber state_sub = n.subscribe("/coax_server/state", 50, stateCallback);
	
	filtered_state_pub = n.advertise<coax_client::CoaxStateFiltered>("/coax_filtered/state", 50);
	
	ros::Rate loop_rate(50);
	
	while(ros::ok())
	{
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}

void stateCallback(const coax_msgs::CoaxStateConstPtr& msg)
{
	coax_client::CoaxStateFiltered new_state;
	
	new_state.header.stamp = ros::Time::now();
	new_state.header.frame_id = "continuous"; 
	
	new_state.ranges[0] = calculateDistance(max(msg->hranges[0],0.05f),DEFAULT_FRONT_SLOPE,DEFAULT_FRONT_OFFSET);
	new_state.ranges[1] = calculateDistance(max(msg->hranges[1],0.05f),DEFAULT_LEFT_SLOPE,DEFAULT_LEFT_OFFSET);
	new_state.ranges[2] = calculateDistance(max(msg->hranges[2],0.05f),DEFAULT_RIGHT_SLOPE,DEFAULT_RIGHT_OFFSET);
	
	filtered_state_pub.publish(new_state);
}

double calculateDistance(const double &sensor_value, const double &slope, const double &offset)
{
	return (slope*log(sensor_value)+offset);
}
