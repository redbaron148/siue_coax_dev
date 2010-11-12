#include "ros/ros.h"
#include "coax_msgs/CoaxState.h"
#include "coax_msgs/CoaxReachNavState.h"
#include "coax_msgs/CoaxSetControl.h"
#include "coax_msgs/CoaxConfigureControl.h"
#include "coax_msgs/CoaxControl.h"

ros::ServiceClient reach_nav_state_client;
ros::ServiceClient configure_control_client;

coax_msgs::CoaxControl global_control;
bool is_dirty;
bool is_ctrlled;

using namespace std;

bool setControl(coax_msgs::CoaxSetControl::Request  &req, 
				coax_msgs::CoaxSetControl::Response &res);
void stateCallback(const coax_msgs::CoaxStateConstPtr& msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coax_range_data");
	ros::NodeHandle n("coax_range_data");
	
	int sensor_num = -1;
	float range = 0.0;
	
	if(n.getParam("/coax_range_data/sensor", sensor_num));
	if(n.getParam("/coax_range_data/range",range));
	
	if(sensor_num < 0 || sensor_num > 2)
	{
		ROS_INFO("sensor_num must be between 0 and 2, set /coax_range_data/sensor to the correct value");
	}
	
	if(range <= 0 || range > 1.5)
	{
		ROS_INFO("/coax_range_data/range must be between 0 and 1.5 meters");
	}
	
	ros::Subscriber state_sub = n.subscribe("/coax_server/state", 50, stateCallback);
	
	return 0;
}

void stateCallback(const coax_msgs::CoaxStateConstPtr& msg)
{
	is_ctrlled = (msg->mode.navigation == 5);
}
