/*
 *  File Name:      BlobPatternFinderNode.cpp
 *  Programmer:     Aaron Parker
 *  Date Made:      02-16-2011
 *  Description:    ROS node, subscribes to /blob_filter/blobs, finds patterns blobs make.
 */

#include <ros/ros.h>
#include <CoaxClientConst.h>
#include <BlobUtilityLibrary.h>
#include <cmvision/Blobs.h>
#include <coax_client/BlobSequence.h>

//global variables
int PUBLISH_FREQ;
int FBLOBS_MSG_BUFFER;
int MSG_QUEUE;

ros::Publisher blob_patt_pub;

using namespace std;

void fBlobsCallback(cmvision::Blobs msg);
void getParams(const ros::NodeHandle &nh);
bool isInList(const unsigned int &i, std::list<unsigned int> &list);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_pattern_finder");
    ros::NodeHandle n("blob_pattern_finder");
    
    getParams(n);
    
    ros::Subscriber filtered_blobs_sub = n.subscribe("/blob_filter/blobs", FBLOBS_MSG_BUFFER, &fBlobsCallback);
    //blob_patt_pub = n.advertise<coax_client::BlobPositions>("/blob_pattern_finder/blobs", MSG_QUEUE);
    
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
    std::cout << "blob count: " << msg.blob_count << std::endl;
    std::vector<std::vector<unsigned int> > blob_clusters;
    int num_adj_blobs = findAllBlobClusters(msg,blob_clusters);
    std::cout << num_adj_blobs << endl;
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
