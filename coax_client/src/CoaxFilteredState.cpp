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
	
	ros::Subscriber state_sub = n.subscribe("/coax_server/state", 50, &stateCallback);
	
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
	static double faccel[3] = {0,0,0};
	double roll = floorf(msg->roll * 100 +  0.5) / 100;
	double pitch = floorf(msg->pitch * 100 +  0.5) / 100;
	double yaw = floorf(msg->yaw * 100 +  0.5) / 100;

	coax_client::CoaxStateFiltered new_state;
	
	new_state.header.stamp = ros::Time::now();
	new_state.header.frame_id = "continuous";

	faccel[0] = .6*faccel[0]+.4*msg->accel[0];
	faccel[1] = .6*faccel[1]+.4*msg->accel[1];
	faccel[2] = .6*faccel[2]+.4*msg->accel[2];

	new_state.accel[0] = faccel[0];
	new_state.accel[1] = faccel[1];
	new_state.accel[2] = faccel[2];

	new_state.global_accel[0] = cos(pitch)*cos(yaw)*faccel[0]-sin(yaw)*cos(pitch)*faccel[1]+sin(pitch)*faccel[2];
	new_state.global_accel[1] = ((cos(yaw)*sin(roll)*sin(pitch)+cos(roll)*sin(yaw))*faccel[0])-
			  ( ( ( sin(yaw)*sin(roll)*sin(pitch) ) - ( cos(roll)*cos(yaw) ) ) *faccel[1])-
			  (sin(roll)*cos(pitch)*faccel[2]);
	new_state.global_accel[2] = (((-sin(pitch)*cos(roll)*cos(yaw))+(sin(roll)*sin(yaw)))*faccel[0])+
			  (((sin(pitch)*sin(yaw)*cos(roll))+(sin(roll)*cos(yaw)))*faccel[1])+
			  (cos(roll)*cos(pitch)*faccel[2]);
	
	new_state.ranges[0] = calculateDistance(max(msg->hranges[0],0.05f),DEFAULT_FRONT_SLOPE,DEFAULT_FRONT_OFFSET);
	new_state.ranges[1] = calculateDistance(max(msg->hranges[1],0.05f),DEFAULT_LEFT_SLOPE,DEFAULT_LEFT_OFFSET);
	new_state.ranges[2] = calculateDistance(max(msg->hranges[2],0.05f),DEFAULT_RIGHT_SLOPE,DEFAULT_RIGHT_OFFSET);
	
	filtered_state_pub.publish(new_state);
}

double calculateDistance(const double &sensor_value, const double &slope, const double &offset)
{
	return (slope*log(sensor_value)+offset);
}
