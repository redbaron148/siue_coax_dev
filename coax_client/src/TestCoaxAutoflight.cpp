#include "ros/ros.h"
#include "coax_msgs/CoaxState.h"
#include "coax_msgs/CoaxReachNavState.h"
#include <iostream>

ros::ServiceClient reach_nav_state_client;

using namespace std;

void state_subscriber_callback(const coax_msgs::CoaxStateConstPtr& msg)
{
	cout << "recieved a state!\nfront ET: " << msg->hranges[0] << endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_coax_client");
	ros::NodeHandle n;

	ros::Subscriber coax_state_subscriber = n.subscribe("coax_server/state",1,state_subscriber_callback);
	reach_nav_state_client = n.serviceClient<coax_msgs::CoaxReachNavState>("coax_server/reach_nav_state");
	
	coax_msgs::CoaxReachNavState nav_state;
	nav_state.request.desiredState = 1;
	nav_state.request.timeout = 5;

	reach_nav_state_client.call(nav_state);
	cout << "reach idle, wait 1 second then go to takeoff\n";

	usleep(1000000);

	nav_state.request.desiredState = 3;
	reach_nav_state_client.call(nav_state);
	cout << "reach " << nav_state.request.desiredState  << ", wait 5 seconds then go to stop mode.\n";

	usleep(5000000);

	nav_state.request.desiredState = 0;
	reach_nav_state_client.call(nav_state);
	cout << "reach stop mode. end of program\n";

	//ros::spinOnce();	

	return 0;
}
