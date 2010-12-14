#include <ros/ros.h>
#include <coax_client/CoaxStateFiltered.h>
#include <coax_client/CoaxLocalization.h>
#include <math.h>

//default values used with the node handler
#define DEFAULT_PUBLISH_FREQ        50
#define DEFAULT_FSTATE_MSG_BUFFER   50
#define DEFAULT_MSG_QUEUE           20
#define DEFAULT_UPDATE_PERIOD       10

//global params, set by getParams function.
int PUBLISH_FREQ;
int FSTATE_MSG_BUFFER;
int MSG_QUEUE;
int UPDATE_PERIOD;

using namespace std;

ros::Publisher filtered_state_pub;

void stateCallback(const coax_client::CoaxStateFilteredConstPtr& msg);
void getParams(const ros::NodeHandle &nh);
double runningAvg(const double &prev, const double &next, const unsigned int &n);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "coax_localization");
	ros::NodeHandle n("coax_localization");

    getParams(n);
	
	ros::Subscriber state_sub = n.subscribe("/coax_filter/state", FSTATE_MSG_BUFFER, &stateCallback);
	
	filtered_state_pub = n.advertise<coax_client::CoaxLocalization>("state", MSG_QUEUE);
	
	ros::Rate loop_rate(PUBLISH_FREQ);
	
	while(ros::ok())
	{
		ros::spinOnce();
        loop_rate.sleep();
	}
	
	return 0;
}

void stateCallback(const coax_client::CoaxStateFilteredConstPtr& msg)
{
    
}

void getParams(const ros::NodeHandle &nh)
{

    //avg accel update period (number of msgs until next localization udpate)
    if (nh.getParam("update_period", UPDATE_PERIOD))
    {
        ROS_INFO("Set update_period to %d.", UPDATE_PERIOD);
    }
    else
    {
        if(nh.hasParam("update_period"))
            ROS_WARN("update_period must be an integer. Setting default value: %d", DEFAULT_UPDATE_PERIOD);
        else
            ROS_WARN("No value set for update_period. Setting default value: %d", DEFAULT_UPDATE_PERIOD);
        UPDATE_PERIOD = DEFAULT_UPDATE_PERIOD;
    }

    //frequency this node publishes a new topic
    if (nh.getParam("publish_freq", PUBLISH_FREQ))
    {
        ROS_INFO("Set publish_freq to %d.", PUBLISH_FREQ);
    }
    else
    {
        if(nh.hasParam("publish_freq"))
            ROS_WARN("publish_freq must be an integer. Setting default value: %d", DEFAULT_PUBLISH_FREQ);
        else
            ROS_WARN("No value set for publish_freq. Setting default value: %d", DEFAULT_PUBLISH_FREQ);
        PUBLISH_FREQ = DEFAULT_PUBLISH_FREQ;
    }

    //number of states from coax_server this node will buffer before it begins to drop them
    if (nh.getParam("fstate_msg_buffer", FSTATE_MSG_BUFFER))
    {
        ROS_INFO("Set fstate_msg_buffer to %d.", FSTATE_MSG_BUFFER);
    }
    else
    {
        if(nh.hasParam("fstate_msg_buffer"))
            ROS_WARN("fstate_msg_buffer must be an integer. Setting default value: %d", DEFAULT_FSTATE_MSG_BUFFER);
        else
            ROS_WARN("No value set for fstate_msg_buffer. Setting default value: %d", DEFAULT_FSTATE_MSG_BUFFER);
        FSTATE_MSG_BUFFER = DEFAULT_FSTATE_MSG_BUFFER;
    }

    //number of messages this node will queue for publishing before droping old data
    if (nh.getParam("msg_queue", MSG_QUEUE))
    {
        ROS_INFO("Set msg_queue to %d.", MSG_QUEUE);
    }
    else
    {
        if(nh.hasParam("msg_queue"))
            ROS_WARN("msg_queue must be an integer. Setting default value: %d", DEFAULT_MSG_QUEUE);
        else
            ROS_WARN("No value set for msg_queue. Setting default value: %d", DEFAULT_MSG_QUEUE);
        MSG_QUEUE = DEFAULT_MSG_QUEUE;
    }
}

double runningAvg(const double &prev, const double &next, const unsigned int &n)
{
    return ((prev*(n-1))+next)/double(n);
}
