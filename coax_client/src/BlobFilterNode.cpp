/*
 *  File Name:      BlobFilterNode.cpp
 *  Programmer:     Aaron Parker
 *  Date Made:      01-17-2010
 *  Description:    ROS node, filters and publishes important blobs under topic 
 *                  "/blob_filter/blobs". Also contains orientation and altitude
 *                  of helicopter.
 *  01-21-2011:     reporting the orientation of the helicopter doesn't make 
 *                  much sense when filtering blobs. was just easier for the 
 *                  project at hand.
 */

#include <ros/ros.h>
#include <CoaxClientConst.h>
#include <cmvision/Blobs.h>
#include <coax_msgs/CoaxState.h>

//global variables
int PUBLISH_FREQ;
int BLOBS_MSG_BUFFER;
int MIN_BLOB_AREA;
int MSG_QUEUE;

using namespace std;

ros::Publisher filtered_blob_pub;

void blobsCallback(cmvision::Blobs msg);
void getParams(const ros::NodeHandle &nh);
cmvision::Blobs filterSmallBlobs(cmvision::Blobs blobs);
bool blobsAreAdjacent(const cmvision::Blob &b1, const cmvision::Blob &b2);
float blobAngle(const cmvision::Blob &b1, const cmvision::Blob &b2);
cmvision::Blobs findAdjacentBlobs(const cmvision::Blob &blob, const cmvision::Blobs &blobs);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_filter");
    ros::NodeHandle n("blob_filter");
    
    getParams(n);

    ros::Subscriber blobs_sub = n.subscribe("/blobs", BLOBS_MSG_BUFFER, &blobsCallback);
    filtered_blob_pub = n.advertise<cmvision::Blobs>("/blob_filter/blobs", MSG_QUEUE);

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
    cmvision::Blobs return_blobs;
    
    return_blobs = filterSmallBlobs(msg);
    
    filtered_blob_pub.publish(msg);
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
    
    //number of messages this node will queue for publishing before it drops data
    if (nh.getParam("min_blob_area", MIN_BLOB_AREA))
    {
        ROS_INFO("Set %s/min_blob_area to %d",nh.getNamespace().c_str(), MIN_BLOB_AREA);
    }
    else
    {
        if(nh.hasParam("min_blob_area"))
          ROS_WARN("%s/min_blob_area must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FBLOB_MIN_BLOB_AREA);
      else
        ROS_WARN("No value set for %s/min_blob_area. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FBLOB_MIN_BLOB_AREA);
      MIN_BLOB_AREA = DEFAULT_FBLOB_MIN_BLOB_AREA;
    }
}

cmvision::Blobs filterSmallBlobs(cmvision::Blobs blobs)
{
    for(int i = blobs.blob_count-1;i>=0;i--)
    {
        if(!(blobs.blobs[i].area >= (unsigned)MIN_BLOB_AREA)) 
        {
            blobs.blobs.erase(blobs.blobs.begin()+i);
            blobs.blob_count--;
        }
    }
    return blobs;
}

bool blobsAreAdjacent(const cmvision::Blob &b1, const cmvision::Blob &b2)
{
    return ((abs((int)(b1.x-b2.x))-3 <= (int)((b1.right-b1.left)/2+(b2.right-b2.left)/2)) &&
            (abs((int)(b1.y-b2.y))-3 <= (int)((b1.bottom-b1.top)/2+(b2.bottom-b2.top)/2)));
}

float blobAngle(const cmvision::Blob &b1, const cmvision::Blob &b2)
{
    return (atan2(b2.y-b1.y,b2.x-b1.x));
}

cmvision::Blobs findAdjacentBlobs(const cmvision::Blob &blob, const cmvision::Blobs &blobs)
{
    cmvision::Blobs adj_blobs;
    for(int i = blobs.blob_count-1;i>=0;i--)
    {
        if(blobsAreAdjacent(blob,blobs.blobs[i])) adj_blobs.blobs.push_back(blobs.blobs[i]);
    }
    return adj_blobs;
}
