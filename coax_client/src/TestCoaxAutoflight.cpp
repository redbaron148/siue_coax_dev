#include "ros/ros.h"
#include "coax_msgs/CoaxState.h"
#include "coax_msgs/CoaxReachNavState.h"
#include "coax_msgs/CoaxSetControl.h"
#include <iostream>

ros::ServiceClient reach_nav_state_client;
ros::ServiceClient set_control_client;

using namespace std;

void state_subscriber_callback(const coax_msgs::CoaxStateConstPtr& msg)
{
	cout << "recieved a state!\nfront ET: " << msg->hranges[0] << endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_coax_client");
	ros::NodeHandle n;

	//ros::Subscriber coax_state_subscriber = n.subscribe("coax_server/state",1,state_subscriber_callback);
	reach_nav_state_client = n.serviceClient<coax_msgs::CoaxReachNavState>("coax_server/reach_nav_state");
	set_control_client = n.serviceClient<coax_msgs::CoaxSetControl>("coax_server/set_control");
	
	coax_msgs::CoaxReachNavState nav_state;
	coax_msgs::CoaxSetControl control;

	nav_state.request.desiredState = 5;
	nav_state.request.timeout = 10;

	control.request.roll = 0;
	control.request.pitch = 0;
	control.request.yaw = 0;
	control.request.altitude = 1.0;

	reach_nav_state_client.call(nav_state);
	cout << "reach controlled state, wait 1 second\n";

	usleep(100000);

	set_control_client.call(control);
	cout << "set new control, wait 1 second\n";

	//usleep(10000000);

	nav_state.request.desiredState = 3;

	reach_nav_state_client.call(nav_state);
	cout << "land, end of program.\n";

	//ros::spinOnce();	

	return 0;
}
