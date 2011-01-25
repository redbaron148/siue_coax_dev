/*
 *  File Name:      BlobPositionTrackerNode.cpp
 *  Programmer:     Aaron Parker
 *  Date Made:      01-17-2010
 *  Description:    ROS node, subscribes to /blob_filter/blobs, calculates 
 *                  position relative to the coax helicopter of every blob in 
 *                  the topic.
 *  01-21-2011:     
 */

#include <ros/ros.h>
#include <CoaxClientConst.h>
#include <coax_msgs/CoaxState.h>
#include <coax_client/BlobPositions.h>
#include <cmvision/Blobs.h>
#include <roslib/Header.h>
#include <vector>

//global variables
double FIELD_OF_VIEW_HORIZ;
double FIELD_OF_VIEW_VERT;
int PUBLISH_FREQ;
int FBLOBS_MSG_BUFFER;
int STATE_MSG_BUFFER;
int MSG_QUEUE;

std::vector<boost::shared_ptr<coax_msgs::CoaxState> > STATE_BUFFER(10,boost::shared_ptr<coax_msgs::CoaxState>(new coax_msgs::CoaxState));

//std::vector< int > int_vector;

ros::Publisher blob_pose_pub;

using namespace std;

void fBlobsCallback(cmvision::Blobs msg);
void stateCallback(boost::shared_ptr<coax_msgs::CoaxState> msg);
void getParams(const ros::NodeHandle &nh);
boost::shared_ptr<coax_msgs::CoaxState> findClosestStampedState(roslib::Header header);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "blob_positions");
	ros::NodeHandle n("blob_positions");
	
	getParams(n);
    
    ros::Subscriber filtered_blobs_sub = n.subscribe("/blob_filter/blobs", FBLOBS_MSG_BUFFER, &fBlobsCallback);
    ros::Subscriber coax_state_sub = n.subscribe("/coax_server/state",STATE_MSG_BUFFER, &stateCallback);
    blob_pose_pub = n.advertise<coax_client::BlobPositions>("/blob_position/blobs", MSG_QUEUE);
    
    //ros::spin();
    
    ros::Rate loop_rate(PUBLISH_FREQ);
    
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
    
	return 0;
}

void fBlobsCallback(cmvision::Blobs msg)
{
    coax_client::BlobPositions blob_poses;
    boost::shared_ptr<coax_msgs::CoaxState> state = findClosestStampedState(msg.header);
    blob_poses.header = msg.header;
    ROS_INFO("state is %f seconds ahead of the fblob.", fabs((state->header.stamp - msg.header.stamp).toSec()));

    float altitude = 0.782;//state->zrange;
    float center_x = msg.image_width/2.0;
    float center_y = msg.image_height/2.0;
    float x_from_center = msg.blobs[0].x-center_x;
    float y_from_center = msg.blobs[0].y-center_y;
    
    float degrees_per_pixel_horiz   = FIELD_OF_VIEW_HORIZ/msg.image_width;
    float degrees_per_pixel_vert    = FIELD_OF_VIEW_VERT/msg.image_height;
    
    float angle_horiz = degrees_per_pixel_horiz*x_from_center;
    float angle_vert  = degrees_per_pixel_vert*y_from_center;
    
    //ROS_INFO("\n\ny: %f\ndegree vert: %f\n",degrees_per_pixel_horiz*x_from_center, degrees_per_pixel_vert*y_from_center);
    
    ROS_INFO("degrees from center horiz: %f",angle_horiz-(state->roll*180.0/3.14159));
    ROS_INFO("degrees from center vert: %f",angle_vert-(state->pitch*180.0/3.14159));
    ROS_INFO("x: %f",altitude*sin((angle_horiz*3.14159/180.0)-state->roll));
    ROS_INFO("y: %f",altitude*sin((angle_vert*3.14159/180.0)-state->pitch));
    
	//ROS_INFO("Size of vec: %d",STATE_BUFFER.size());
    
    //blob_pose_pub.publish(blob_poses);
}

void stateCallback(boost::shared_ptr<coax_msgs::CoaxState> msg)
{
    static int count = 0;
	STATE_BUFFER[count%10] = msg;
	count ++;
}

void getParams(const ros::NodeHandle &nh)
{
    //field of view of the camera onboard the gumstix of the COAX helicopter in degrees.
    if (nh.getParam("field_of_view_horiz", FIELD_OF_VIEW_HORIZ))
    {
        ROS_INFO("Set %s/field_of_view_horiz to %f",nh.getNamespace().c_str(), FIELD_OF_VIEW_HORIZ);
    }
    else
    {
        if(nh.hasParam("field_of_view"))
            ROS_WARN("%s/field_of_view_horiz must be a float. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_FIELD_OF_VIEW_HORIZ);
        else
            ROS_WARN("No value set for %s/field_of_view_horiz. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_FIELD_OF_VIEW_HORIZ);
        FIELD_OF_VIEW_HORIZ = DEFAULT_FIELD_OF_VIEW_HORIZ;
    }
    
    //field of view of the camera onboard the gumstix of the COAX helicopter in degrees.
    if (nh.getParam("field_of_view_vert", FIELD_OF_VIEW_VERT))
    {
        ROS_INFO("Set %s/field_of_view_vert to %f",nh.getNamespace().c_str(), FIELD_OF_VIEW_VERT);
    }
    else
    {
        if(nh.hasParam("field_of_view_vert"))
            ROS_WARN("%s/field_of_view_vert must be a float. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_FIELD_OF_VIEW_VERT);
        else
            ROS_WARN("No value set for %s/field_of_view_vert. Setting default value: %f",nh.getNamespace().c_str(), DEFAULT_FIELD_OF_VIEW_VERT);
        FIELD_OF_VIEW_VERT = DEFAULT_FIELD_OF_VIEW_VERT;
    }
    
    //frequency this node publishes a new topic
	if(nh.getParam("publish_freq", PUBLISH_FREQ))
	{
		ROS_INFO("Set %s/publish_freq to %d",nh.getNamespace().c_str(), PUBLISH_FREQ);
	}
	else
	{
		if(nh.hasParam("publish_freq"))
			ROS_WARN("%s/publish_freq must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POS_NODE_PUBLISH_FREQ);
	  else
		  ROS_WARN("No value set for %s/publish_freq. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POS_NODE_PUBLISH_FREQ);
	  PUBLISH_FREQ = DEFAULT_BLOB_POS_NODE_PUBLISH_FREQ;
	}

	//number of filtered blobs from blob_filter this node will buffer before it begins to drop them
	if (nh.getParam("filtered_blobs_msg_buffer", FBLOBS_MSG_BUFFER))
	{
		ROS_INFO("Set %s/filtered_blobs_msg_buffer to %d",nh.getNamespace().c_str(), FBLOBS_MSG_BUFFER);
	}
	else
	{
		if(nh.hasParam("filtered_blobs_msg_buffer"))
			ROS_WARN("%s/filtered_blobs_msg_buffer must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POS_NODE_FBLOBS_MSG_BUFFER);
	  else
		  ROS_WARN("No value set for %s/filtered_blobs_msg_buffer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POS_NODE_FBLOBS_MSG_BUFFER);
	  FBLOBS_MSG_BUFFER = DEFAULT_BLOB_POS_NODE_FBLOBS_MSG_BUFFER;
	}
	
	//number of states from coax_server this node will buffer before it begins to drop them
	if (nh.getParam("state_msg_buffer", STATE_MSG_BUFFER))
	{
		ROS_INFO("Set %s/state_msg_buffer to %d",nh.getNamespace().c_str(), STATE_MSG_BUFFER);
	}
	else
	{
		if(nh.hasParam("state_msg_buffer"))
			ROS_WARN("%s/state_msg_buffer must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POS_NODE_STATE_MSG_BUFFER);
	  else
		  ROS_WARN("No value set for %s/state_msg_buffer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POS_NODE_STATE_MSG_BUFFER);
	  STATE_MSG_BUFFER = DEFAULT_BLOB_POS_NODE_STATE_MSG_BUFFER;
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

boost::shared_ptr<coax_msgs::CoaxState> findClosestStampedState(roslib::Header header)
{
    int closest = 0;
    float best_time = fabs((STATE_BUFFER[0]->header.stamp-header.stamp).toSec());
    for(int i = 1;i < 10;i++)
    {
        if(fabs((STATE_BUFFER[i]->header.stamp-header.stamp).toSec()) <= best_time)
        {
            best_time = fabs((STATE_BUFFER[closest]->header.stamp-header.stamp).toSec());
            closest = i;
        }
    }
    return STATE_BUFFER[closest];
}
