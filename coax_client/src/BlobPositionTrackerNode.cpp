/*
 *  File Name:      CoaxServerNode.cpp
 *  Programmer:     Aaron Parker
 *  Date Made:      01-17-2010
 *  Description:    ROS node, subscribes to /blob_filter/blobs, calculates 
 *                  position relative to the coax helicopter of every blob in 
 *                  the topic.
 */

#include <ros/ros.h>
#include <CoaxClientConst.h>
#include <coax_client/BlobPositions.h>

//global variables
double FIELD_OF_VIEW_HORIZ;
double FIELD_OF_VIEW_VERT;
int PUBLISH_FREQ;
int FBLOBS_MSG_BUFFER;
int MSG_QUEUE;

ros::Publisher blob_position_pub;

using namespace std;

void getParams(const ros::NodeHandle &nh);
void filteredBlobsCallback(coax_client::FilteredBlobs msg);
bool 

int main(int argc, char **argv)
{
	ros::init(argc, argv, "blob_positions");
	ros::NodeHandle n("blob_positions");
	
	getParams(n);

    ros::Subscriber filtered_blobs_sub = n.subscribe("/blob_filter/blobs", FBLOBS_MSG_BUFFER, &fBlobsCallback);

    filtered_blob_pub = n.advertise<coax_client::FilteredBlobs>("/blob_positions/blobs", MSG_QUEUE);
    
    getParams(n);

    ros::Rate loop_rate(PUBLISH_FREQ);
    
	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

void filteredBlobsCallback(cmvision::Blobs msg)
{
    coax_client::BlobPositions blob_poses;
    blob_poses.header = msg.header;
    blob_poses.blobs = msg.blobs;
    
    
    /*float altitude = 30;
    float x = (req.blobs.blobs[req.blob_num].right+req.blobs.blobs[req.blob_num].left)/2.0;
    float y = (req.blobs.blobs[req.blob_num].top+req.blobs.blobs[req.blob_num].bottom)/2.0;
    unsigned short int width = req.blobs.image_width;
    unsigned short int height = req.blobs.image_height;
    float center_x = width/2.0;
    float center_y = height/2.0;
    float x_from_center = x-center_x;
    float y_from_center = y-center_y;
    
    float degrees_per_pixel_horiz   = FIELD_OF_VIEW_HORIZ/width;
    float degrees_per_pixel_vert    = FIELD_OF_VIEW_VERT/height;
    
    float angle_horiz = degrees_per_pixel_horiz*x_from_center;
    float angle_vert  = degrees_per_pixel_vert*y_from_center;
    
    //ROS_INFO("\n\ny: %f\ndegree vert: %f\n",degrees_per_pixel_horiz*x_from_center, degrees_per_pixel_vert*y_from_center);
    
    ROS_INFO("x: %f\n",altitude*tan(angle_horiz*3.14159/180.));
    
    return true;*/
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

	//number of states from coax_server this node will buffer before it begins to drop them
	if (nh.getParam("blobs_msg_buffer", FBLOBS_MSG_BUFFER))
	{
		ROS_INFO("Set %s/blobs_msg_buffer to %d",nh.getNamespace().c_str(), FBLOBS_MSG_BUFFER);
	}
	else
	{
		if(nh.hasParam("blobs_msg_buffer"))
			ROS_WARN("%s/blobs_msg_buffer must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POS_NODE_FBLOBS_MSG_BUFFER);
	  else
		  ROS_WARN("No value set for %s/blobs_msg_buffer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POS_NODE_FBLOBS_MSG_BUFFER);
	  FBLOBS_MSG_BUFFER = DEFAULT_BLOB_POS_NODE_FBLOBS_MSG_BUFFER;
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
