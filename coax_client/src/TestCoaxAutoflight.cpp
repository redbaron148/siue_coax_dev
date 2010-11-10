#include "ros/ros.h"
#include "coax_msgs/CoaxState.h"
#include "coax_msgs/CoaxReachNavState.h"
#include "coax_msgs/CoaxSetControl.h"
#include "coax_msgs/CoaxConfigureControl.h"
#include <iostream>

ros::ServiceClient reach_nav_state_client;
ros::ServiceClient set_control_client;
ros::ServiceClient configure_control_client;

using namespace std;

void state_subscriber_callback(const coax_msgs::CoaxStateConstPtr& msg);
int configureControl(const int &rollMode, const int &pitchMode, const int &yawMode, const int &altitudeMode);
int reachNavState(const int &navState, const double &timeout);
int setControl(const double &roll, const double &pitch, const double &yaw, const double &altitude);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_coax_client");
	ros::NodeHandle n;

	ros::Subscriber coax_state_subscriber = n.subscribe("coax_server/state",1,state_subscriber_callback);
	reach_nav_state_client = n.serviceClient<coax_msgs::CoaxReachNavState>("coax_server/reach_nav_state");
	set_control_client = n.serviceClient<coax_msgs::CoaxSetControl>("coax_server/set_control");
	configure_control_client = n.serviceClient<coax_msgs::CoaxConfigureControl>("coax_server/configure_control");

	configureControl(1,1,1,1);
	setControl(0,0,0,2);
	//ROS_INFO("set new control parameters");
	
	ros::spinOnce();
	
	sleep(1);
	
	reachNavState(5,25);
	//usleep(10000000);
	
	for(int i = 1;i <= 50 && ros::ok();i++)
	{
		setControl(0,-.1,0,1);
		ROS_INFO("keep alive block. %d",i);
		ros::spinOnce();
		usleep(200000);
	}
	
	reachNavState(0,10);
	
	return 0;
}

void state_subscriber_callback(const coax_msgs::CoaxStateConstPtr& msg)
{
	ROS_INFO("recieved a state!\nfront ET: %f\nleft  ET: %f\nright ET: %f\naltitude: %f\npressure: %f\nz accel : %f\ny accel : %f\nz accel : %f",msg->hranges[0],msg->hranges[1],msg->hranges[2],msg->zfiltered,msg->pressure,msg->accel[0],msg->accel[1],msg->accel[2]);
}

int configureControl(const int &rollMode, const int &pitchMode, const int &yawMode, const int &altitudeMode)
{
	coax_msgs::CoaxConfigureControl configure_control;

	configure_control.request.rollMode 	= rollMode;
	configure_control.request.pitchMode	= pitchMode;
	configure_control.request.yawMode 	= yawMode;
	configure_control.request.altitudeMode 	= altitudeMode;

	configure_control_client.call(configure_control);

	return configure_control.response.result;
}

int reachNavState(const int &navState, const double &timeout)
{
	coax_msgs::CoaxReachNavState nav_state;

	nav_state.request.desiredState = navState;
	nav_state.request.timeout = timeout;

	reach_nav_state_client.call(nav_state);

	return nav_state.response.result;
}

int setControl(const double &roll, const double &pitch, const double &yaw, const double &altitude)
{
	coax_msgs::CoaxSetControl control;

	control.request.roll = roll;
	control.request.pitch = pitch;
	control.request.yaw = yaw;
	control.request.altitude = altitude;

	set_control_client.call(control);

	return control.response.result;
}
