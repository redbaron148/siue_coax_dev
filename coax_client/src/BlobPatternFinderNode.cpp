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
#include <iostream>

//global variables
int PUBLISH_FREQ;
int FBLOBS_MSG_BUFFER;
int MSG_QUEUE;

ros::Publisher blob_patt_pub;

using namespace std;

void fBlobsCallback(cmvision::Blobs msg);
void getParams(const ros::NodeHandle &nh);
void filterBlobClusters(std::vector<std::vector<unsigned int> > &clusters)
{
   // cout << "number of clusters is " << clusters.size() << endl;
    for(unsigned int i = 0;i<clusters.size();i++)
    {
        //cout << "    checking " << i << " " << endl;
        if(clusters[i].size()!=BLOB_SEQUENCE_SIZE)
        {
            /*for(int j = 0;j<clusters[i].size();j++)
                cout << "            members: " << clusters[i][j] << " ";*/
            clusters.erase(clusters.begin()+i);
            //cout << "\n        removed cluster because of size: " << i << endl;
            i--;
        }
        //else cout << "        no problem found" << endl;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_pattern_finder");
    ros::NodeHandle n("blob_pattern_finder");
    
    getParams(n);
    
    ros::Subscriber filtered_blobs_sub = n.subscribe("/blob_filter/blobs", FBLOBS_MSG_BUFFER, &fBlobsCallback);
    blob_patt_pub = n.advertise<coax_client::BlobSequences>("/blob_pattern_finder/blob_sequences", MSG_QUEUE);
    
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
    if(msg.blobs.size()>=4)
    {
        coax_client::BlobSequences sequences;
        sequences.image_height = msg.image_height;
        sequences.image_width = msg.image_width;
        sequences.header = msg.header;
        std::vector<std::vector<unsigned int> > blob_clusters;
        findAllBlobClusters(msg,blob_clusters);
        filterBlobClusters(blob_clusters);
        for(unsigned int i = 0;i<blob_clusters.size();i++)
        {
            coax_client::BlobSequence sequence;
            blobSequenceFromCluster(sequence,blob_clusters[i],msg);
            sequences.blob_sequences.push_back(sequence);
        }
        if(sequences.blob_sequences.size())
            blob_patt_pub.publish(sequences);
        //cout << "image_height: " << sequences.image_height << endl;
        //cout << "image_width: " << sequences.image_width << endl;
    }
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
            ROS_WARN("%s/publish_freq must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_PATT_NODE_PUBLISH_FREQ);
        else
            ROS_WARN("No value set for %s/publish_freq. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_PATT_NODE_PUBLISH_FREQ);
        PUBLISH_FREQ = DEFAULT_BLOB_PATT_NODE_PUBLISH_FREQ;
    }

    //number of filtered blobs from blob_filter this node will buffer before it begins to drop them
    if (nh.getParam("filtered_blobs_msg_buffer", FBLOBS_MSG_BUFFER))
    {
        ROS_INFO("Set %s/filtered_blobs_msg_buffer to %d",nh.getNamespace().c_str(), FBLOBS_MSG_BUFFER);
    }
    else
    {
        if(nh.hasParam("filtered_blobs_msg_buffer"))
            ROS_WARN("%s/filtered_blobs_msg_buffer must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_PATT_NODE_FBLOBS_MSG_BUFFER);
        else
            ROS_WARN("No value set for %s/filtered_blobs_msg_buffer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_PATT_NODE_FBLOBS_MSG_BUFFER);
        FBLOBS_MSG_BUFFER = DEFAULT_BLOB_PATT_NODE_FBLOBS_MSG_BUFFER;
    }
    
    //number of messages this node will queue for publishing before it drops data
    if (nh.getParam("msg_queue", MSG_QUEUE))
    {
        ROS_INFO("Set %s/msg_queue to %d",nh.getNamespace().c_str(), MSG_QUEUE);
    }
    else
    {
        if(nh.hasParam("msg_queue"))
          ROS_WARN("%s/msg_queue must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_PATT_NODE_MSG_QUEUE);
      else
        ROS_WARN("No value set for %s/msg_queue. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_PATT_NODE_MSG_QUEUE);
      MSG_QUEUE = DEFAULT_BLOB_PATT_NODE_MSG_QUEUE;
    }
}