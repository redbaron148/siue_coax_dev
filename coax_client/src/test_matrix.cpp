#include "ros/ros.h"
#include "coax_msgs/CoaxState.h"
#include "coax_client/CoaxStateFiltered.h"
#include <math.h>
#include <LinearMath/btMatrix3x3.h>

#define DEFAULT_FRONT_SLOPE  -41.4835
#define DEFAULT_FRONT_OFFSET  50.7125
#define DEFAULT_LEFT_SLOPE   -39.1334
#define DEFAULT_LEFT_OFFSET   48.1207
#define DEFAULT_RIGHT_SLOPE  -38.8275
#define DEFAULT_RIGHT_OFFSET  47.5877

using namespace std;

ros::Publisher filtered_state_pub;

void stateCallback(const coax_msgs::CoaxStateConstPtr& msg);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coax_filtered");
	ros::NodeHandle n("/coax_filtered");
	
	ros::Subscriber state_sub = n.subscribe("/coax_server/state", 50, &stateCallback);

	ros::spin();
}

void stateCallback(const coax_msgs::CoaxStateConstPtr& msg)
{
	//btMatrix3x3::btMatrix3x3 rotz(cos(msg->yaw),-sin(msg->yaw),0,sin(msg->yaw),cos(msg->yaw),0,0,0,1);
	//btMatrix3x3::btMatrix3x3 roty(cos(msg->pitch ),0,sin(msg->pitch ),0,1,0,-sin(msg->pitch ),0,cos(msg->pitch ));
	//btMatrix3x3::btMatrix3x3 rotx(1,0,0,0,cos(msg->roll),-sin(msg->roll),0,sin(msg->roll),cos(msg->roll));

	//btVector3::btVector3 point(msg->accel[0],msg->accel[1],msg->accel[2]);

	//point = rotx*roty*rotz*point;

	double roll = floorf(msg->roll * 100 +  0.5) / 100;
	double pitch = floorf(msg->pitch * 100 +  0.5) / 100;
	double yaw = floorf(msg->yaw * 100 +  0.5) / 100;
	
	//cout << "x = " << point[0] << endl;
	cout << "x = " << cos(pitch)*cos(yaw)*msg->accel[0]-sin(yaw)*cos(pitch)*msg->accel[1]+sin(pitch)*msg->accel[2] << endl;
	//cout << "y = " << point[1] << endl;
	cout << "y = " << ((cos(yaw)*sin(roll)*sin(pitch)+cos(roll)*sin(yaw))*msg->accel[0])-
			  ( ( ( sin(yaw)*sin(roll)*sin(pitch) ) - ( cos(roll)*cos(yaw) ) ) *msg->accel[1])-
			  (sin(roll)*cos(pitch)*msg->accel[2]) << endl;
	//cout << "z = " << point[2] << endl;
	cout << "z = " << (((-sin(pitch)*cos(roll)*cos(yaw))+(sin(roll)*sin(yaw)))*msg->accel[0])+
			  (((sin(pitch)*sin(yaw)*cos(roll))+(sin(roll)*cos(yaw)))*msg->accel[1])+
			  (cos(roll)*cos(pitch)*msg->accel[2]);

	/*cout << (cos(msg->pitch)*cos(msg->yaw)*msg->accel[0])-
		(((cos(msg->roll)*sin(msg->yaw))+(sin(msg->roll)*sin(msg->pitch)*cos(msg->yaw)))*msg->accel[1])+
		(((sin(msg->roll)*sin(msg->yaw))+(cos(msg->roll)*sin(msg->pitch)*cos(msg->yaw)))*msg->accel[2]) << endl;*/

	cout << endl;
	ROS_INFO("bwah!");
}


