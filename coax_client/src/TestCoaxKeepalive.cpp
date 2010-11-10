#include "ros/ros.h"
#include "coax_msgs/CoaxReachNavState.h"
#include <iostream>

ros::ServiceClient reach_nav_state_client;

using namespace std;

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
	control.request.yaw = -1.570;
	control.request.altitude = .5;

	reach_nav_state_client.call(nav_state);
	cout << "reach controlled state, setting new control\n";

	set_control_client.call(control);
	cout << "set new control\n";

	cout << "slept for a second, setting new control.\n";

	cout << "setting state to idle\n";

	nav_state.request.desiredState = 1;

	reach_nav_state_client.call(nav_state);
	cout << "reached land state, end of program.\n";

	//ros::spinOnce();	

	return 0;
}
