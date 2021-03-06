#include "ros/ros.h"
#include "coax_msgs/CoaxState.h"
#include <iostream>

using namespace std;

void state_subscriber_callback(const coax_msgs::CoaxStateConstPtr& msg)
{
    cout << "recieved a state!\nfront ET: " << msg->hranges[FRONT] << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_coax_client");
    ros::NodeHandle n;
    
    ros::Subscriber coax_state_subscriber = n.subscribe("coax_server/state",1,state_subscriber_callback);
    ros::spin();

    return 0;
}
