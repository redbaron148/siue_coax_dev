/*
 *  File Name:      BlobFilterNode.cpp
 *  Programmer:     Aaron Parker
 *  Date Made:      01-17-2010
 *  Description:    ROS node, filters and publishes important blobs under topic 
 *                  "/blob_filter/blobs". Also contains orientation and altitude
 *                  of helicopter.
 */

#include <ros/ros.h>
#include <CoaxClientConst.h>
#include <cmvision/Blobs.h>
#include <coax_msgs/CoaxState.h>

//global variables
int PUBLISH_FREQ;
int STATE_MSG_BUFFER;
int BLOBS_MSG_BUFFER;
int MSG_QUEUE;

using namespace std;

ros::Publisher filtered_blob_pub;

void blobsCallback(cmvision::Blobs msg);
void stateCallback(boost::shared_ptr<coax_msgs::CoaxState> msg);
void getParams(const ros::NodeHandle &nh);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_filter");
    ros::NodeHandle n("blob_filter");

    ros::Subscriber state_sub = n.subscribe("/coax_server/state", STATE_MSG_BUFFER, &stateCallback);
    ros::Subscriber blobs_sub = n.subscribe("/blobs", BLOBS_MSG_BUFFER, &blobsCallback);

    filtered_blob_pub = n.advertise<coax_client::FilteredBlobs>("/blob_filter/blobs", MSG_QUEUE);
    getParams(n);

    ros::Rate loop_rate(PUBLISH_FREQ);
    
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void blobsCallback(cmvision::Blobs msg)
{
    if(msg.blob_count > 0)
    {
        filtered_blob_pub.publish(msg);
    }
}

void stateCallback(boost::shared_ptr<coax_msgs::CoaxState> msg)
{
    cur_alt = msg.zfiltered;
}

void getParams(const ros::NodeHandle &nh)
{
	//frequency this node publishes a new topic
	if(nh.getParam("publish_freq", PUBLISH_FREQ))
	{
		ROS_INFO("Set %s/publish_freq to %d",nh.getNamespace().c_str(), PUBLISH_FREQ);
	}
	else
	{
		if(nh.hasParam("publish_freq"))
			ROS_WARN("%s/publish_freq must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FBLOB_NODE_PUBLISH_FREQ);
	  else
		  ROS_WARN("No value set for %s/publish_freq. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FBLOB_NODE_PUBLISH_FREQ);
	  PUBLISH_FREQ = DEFAULT_FBLOB_NODE_PUBLISH_FREQ;
	}
	
	//number of states from coax_server this node will buffer before it begins to drop them
	if (nh.getParam("state_msg_buffer", STATE_MSG_BUFFER))
	{
		ROS_INFO("Set %s/state_msg_buffer to %d",nh.getNamespace().c_str(), STATE_MSG_BUFFER);
	}
	else
	{
		if(nh.hasParam("state_msg_buffer"))
			ROS_WARN("%s/state_msg_buffer must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FBLOB_NODE_STATE_MSG_BUFFER);
		else
			ROS_WARN("No value set for %s/state_msg_buffer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FBLOB_NODE_STATE_MSG_BUFFER);
		STATE_MSG_BUFFER = DEFAULT_FBLOB_NODE_STATE_MSG_BUFFER;
	}

	//number of states from coax_server this node will buffer before it begins to drop them
	if (nh.getParam("blobs_msg_buffer", BLOBS_MSG_BUFFER))
	{
		ROS_INFO("Set %s/blobs_msg_buffer to %d",nh.getNamespace().c_str(), BLOBS_MSG_BUFFER);
	}
	else
	{
		if(nh.hasParam("blobs_msg_buffer"))
			ROS_WARN("%s/blobs_msg_buffer must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FBLOB_NODE_BLOBS_MSG_BUFFER);
	  else
		  ROS_WARN("No value set for %s/blobs_msg_buffer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FBLOB_NODE_BLOBS_MSG_BUFFER);
	  BLOBS_MSG_BUFFER = DEFAULT_FBLOB_NODE_BLOBS_MSG_BUFFER;
	}
	
	//number of messages this node will queue for publishing before it drops data
	if (nh.getParam("msg_queue", MSG_QUEUE))
	{
		ROS_INFO("Set %s/msg_queue to %d",nh.getNamespace().c_str(), MSG_QUEUE);
	}
	else
	{
		if(nh.hasParam("msg_queue"))
		  ROS_WARN("%s/msg_queue must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FBLOB_NODE_MSG_QUEUE);
	  else
	    ROS_WARN("No value set for %s/msg_queue. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FBLOB_NODE_MSG_QUEUE);
	  MSG_QUEUE = DEFAULT_FBLOB_NODE_MSG_QUEUE;
	}
}
