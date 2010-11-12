#include "ros/ros.h"
#include "coax_msgs/CoaxState.h"
#include "coax_client/CoaxStateFiltered.h"

#define DEFAULT_FRONT_SLOPE  -41.4835
#define DEFAULT_FRONT_OFFSET  50.7125
#define DEFAULT_LEFT_SLOPE   -39.1334
#define DEFAULT_LEFT_OFFSET   48.1207
#define DEFAULT_RIGHT_SLOPE  -38.8275
#define DEFAULT_RIGHT_OFFSET  47.5877

using namespace std;

void stateCallback(const coax_msgs::CoaxStateConstPtr& msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coax_filtered");
	ros::NodeHandle n("/coax_filtered");
	
	ros::Subscriber state_sub = n.subscribe("/coax_server/state", 50, stateCallback);
	
	ros::Publisher control_pub = n.advertise<coax_client::CoaxStateFiltered>("/coax_filtered/state", 50);
	
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
	h
}