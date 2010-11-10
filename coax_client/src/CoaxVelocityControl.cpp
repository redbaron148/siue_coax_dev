#include "ros/ros.h"
#include "coax_msgs/CoaxState.h"
#include "coax_msgs/CoaxReachNavState.h"
#include "coax_msgs/CoaxSetControl.h"
#include "coax_msgs/CoaxConfigureControl.h"
#include "coax_msgs/CoaxControl.h"

ros::ServiceClient reach_nav_state_client;
ros::ServiceClient configure_control_client;

#define DEFAULT_ROLL     0.0
#define DEFAULT_PITCH    0.0
#define DEFAULT_YAW      0.0
#define DEFAULT_ALTITUDE 0.5

coax_msgs::CoaxControl global_control;
bool is_dirty;
bool is_ctrlled;

using namespace std;

bool setControl(coax_msgs::CoaxSetControl::Request  &req, 
				coax_msgs::CoaxSetControl::Response &res);
void stateCallback(const coax_msgs::CoaxStateConstPtr& msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coax_velocity_control");
	ros::NodeHandle n("/coax_velocity_control");
	
	ros::Subscriber state_sub = n.subscribe("/coax_server/state", 50, stateCallback);
	
	ros::Publisher control_pub = n.advertise<coax_msgs::CoaxControl>("/coax_server/control", 1);
	ros::ServiceServer set_control_service = n.advertiseService("set_control", setControl);
	
	ros::Rate loop_rate(20);
	
	global_control.roll = 0;
	global_control.pitch = 0;
	global_control.yaw = 0;
	global_control.altitude = .5;
	is_dirty = false;
	is_ctrlled = false;
	
	while(ros::ok())
	{
		if(is_ctrlled)
		{
			coax_msgs::CoaxControl tmp_control;
			
			tmp_control.roll = global_control.roll;
			tmp_control.pitch = global_control.pitch;
			tmp_control.yaw = global_control.yaw;
			tmp_control.altitude = global_control.altitude;
			
			control_pub.publish(tmp_control);
		
			if(is_dirty)
			{
				ROS_INFO("Published new control.");
				is_dirty = false;
			}
		}
		loop_rate.sleep();
		ros::spinOnce();
	}
	
	return 0;
}

bool setControl(coax_msgs::CoaxSetControl::Request  &req, 
				coax_msgs::CoaxSetControl::Response &res)
{
	if(is_ctrlled)
	{
		global_control.roll = req.roll;
		global_control.pitch = req.pitch;
		global_control.yaw = req.yaw;
		global_control.altitude = req.altitude;
	
		is_dirty = true;
	}
	else
	{
		global_control.roll = DEFAULT_ROLL;
		global_control.pitch = DEFAULT_PITCH;
		global_control.yaw = DEFAULT_YAW;
		global_control.altitude = DEFAULT_ALTITUDE;
	}
	
	return is_dirty;
}

void stateCallback(const coax_msgs::CoaxStateConstPtr& msg)
{
	is_ctrlled = (msg->mode.navigation == 5);
	//ROS_INFO("is_ctrlled: %d", is_ctrlled);
}
