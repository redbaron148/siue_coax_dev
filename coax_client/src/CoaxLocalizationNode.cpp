#include <ros/ros.h>
#include <coax_client/CoaxFilteredState.h>
#include <coax_client/CoaxLocalization.h>
#include <CoaxClientConst.h>
#include <math.h>

//global 
int PUBLISH_FREQ;
int FSTATE_MSG_BUFFER;
int MSG_QUEUE;
int UPDATE_PERIOD;
ros::Publisher filtered_state_pub;

using namespace std;

void stateCallback(boost::shared_ptr<coax_client::CoaxFilteredState> msg);
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

void stateCallback(boost::shared_ptr<coax_client::CoaxFilteredState> msg)
{
    static boost::shared_ptr<coax_client::CoaxLocalization> prev_msg;
    static float running_avg_accel[3] = {0};
    static int tmp = 0;
    float delta_time = 0.;
    tmp = (tmp%UPDATE_PERIOD)+1;

    for(int i = 0;i<3;i++)
    {
        running_avg_accel[i] = runningAvg(running_avg_accel[i],msg->global_accel[i],tmp);
    }
    
    if(tmp == UPDATE_PERIOD)
    {
        running_avg_accel[2] += 9.81;
        boost::shared_ptr<coax_client::CoaxLocalization> new_msg(new coax_client::CoaxLocalization);
        
        if(prev_msg != NULL)
            delta_time = msg->header.stamp.toSec()-prev_msg->header.stamp.toSec();
        else
            prev_msg = new_msg;
            
        new_msg->header = msg->header;
        for(int i=0;i<3;i++)
        {
            //running_avg_accel[i] = floorf(running_avg_accel[i] * 100 +  0.5) / 100;
            new_msg->global_accel_avg[i]=running_avg_accel[i];
            new_msg->global_vel_avg[i]=(running_avg_accel[i]*delta_time)+prev_msg->global_vel_avg[i];
            new_msg->position[i] = (.5*running_avg_accel[i]*delta_time*delta_time)+(new_msg->global_vel_avg[i]*delta_time)+prev_msg->position[i];
            running_avg_accel[i]=0;
        }
        filtered_state_pub.publish(new_msg);
        prev_msg = new_msg;
    }
}

void getParams(const ros::NodeHandle &nh)
{
    //avg accel update period (number of msgs until next localization udpate)
    if (nh.getParam("update_period", UPDATE_PERIOD))
    {
        ROS_INFO("Set %s/update_period to %d",nh.getNamespace().c_str(), UPDATE_PERIOD);
    }
    else
    {
        if(nh.hasParam("update_period"))
            ROS_WARN("%s/update_period must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_LOC_NODE_UPDATE_PERIOD);
        else
            ROS_WARN("No value set for %s/update_period. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_LOC_NODE_UPDATE_PERIOD);
        UPDATE_PERIOD = DEFAULT_LOC_NODE_UPDATE_PERIOD;
    }

    //frequency this node publishes a new topic
    if (nh.getParam("publish_freq", PUBLISH_FREQ))
    {
        ROS_INFO("Set %s/publish_freq to %d",nh.getNamespace().c_str(), PUBLISH_FREQ);
    }
    else
    {
        if(nh.hasParam("publish_freq"))
            ROS_WARN("%s/publish_freq must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_LOC_NODE_PUBLISH_FREQ);
        else
            ROS_WARN("No value set for %s/publish_freq. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_LOC_NODE_PUBLISH_FREQ);
        PUBLISH_FREQ = DEFAULT_LOC_NODE_PUBLISH_FREQ;
    }

    //number of states from coax_filter this node will buffer before it begins to drop them
    if (nh.getParam("fstate_msg_buffer", FSTATE_MSG_BUFFER))
    {
        ROS_INFO("Set %s/fstate_msg_buffer to %d",nh.getNamespace().c_str(), FSTATE_MSG_BUFFER);
    }
    else
    {
        if(nh.hasParam("fstate_msg_buffer"))
            ROS_WARN("%s/fstate_msg_buffer must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_LOC_NODE_FSTATE_MSG_BUFFER);
        else
            ROS_WARN("No value set for %s/fstate_msg_buffer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_LOC_NODE_FSTATE_MSG_BUFFER);
        FSTATE_MSG_BUFFER = DEFAULT_LOC_NODE_FSTATE_MSG_BUFFER;
    }

    //number of messages this node will queue for publishing before droping old data
    if (nh.getParam("msg_queue", MSG_QUEUE))
    {
        ROS_INFO("Set %s/msg_queue to %d",nh.getNamespace().c_str(), MSG_QUEUE);
    }
    else
    {
        if(nh.hasParam("msg_queue"))
            ROS_WARN("%s/msg_queue must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_LOC_NODE_MSG_QUEUE);
        else
            ROS_WARN("No value set for %s/msg_queue. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_LOC_NODE_MSG_QUEUE);
        MSG_QUEUE = DEFAULT_LOC_NODE_MSG_QUEUE;
    }
}

double runningAvg(const double &prev, const double &next, const unsigned int &n)
{
    return ((prev*(n-1))+next)/double(n);
}
