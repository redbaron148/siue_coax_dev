#include <ros/ros.h>
#include <coax_msgs/CoaxState.h>
#include <coax_client/CoaxStateFiltered.h>
#include <tf/transform_broadcaster.h>

using namespace std;

string node_name;

void stateCallback(const coax_msgs::CoaxStateConstPtr& msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coax_filtered");
	ros::NodeHandle n("/coax_filtered");

	if (argc != 2){ROS_ERROR("need node name as argument"); return -1;};
  	node_name = argv[1];
	
	ros::Subscriber state_sub = n.subscribe("/coax_server/state", 50, &stateCallback);
	
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
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(msg->accel[0],msg->accel[1],msg->accel[2]) );
	transform.setRotation( tf::Quaternion(msg->roll, msg->pitch, msg->yaw) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", node_name));
}
