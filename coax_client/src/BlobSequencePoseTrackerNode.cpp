/*
 *  File Name:      BlobPositionTrackerNode.cpp
 *  Programmer:     Aaron Parker
 *  Date Made:      01-17-2010
 *  Description:    ROS node, subscribes to /blob_filter/blobs, calculates 
 *                  position relative to the coax helicopter of every blob in 
 *                  the topic.
 */

#include <ros/ros.h>
#include <CoaxClientConst.h>
#include <coax_msgs/CoaxState.h>
#include <coax_client/BlobSequencePoses.h>
#include <coax_client/BlobSequences.h>
#include <cmvision/Blobs.h>

//global variables
double FIELD_OF_VIEW_HORIZ;
double FIELD_OF_VIEW_VERT;
int PUBLISH_FREQ;
int SEQ_MSG_BUFFER;
int STATE_MSG_BUFFER;
int MSG_QUEUE;

std::vector<boost::shared_ptr<coax_msgs::CoaxState> > STATE_BUFFER(10,boost::shared_ptr<coax_msgs::CoaxState>(new coax_msgs::CoaxState));
ros::Publisher seq_pose_pub;

using namespace std;

void blobSequencesCallback(coax_client::BlobSequences msg);
void stateCallback(boost::shared_ptr<coax_msgs::CoaxState> msg);
void getParams(const ros::NodeHandle &nh);
boost::shared_ptr<coax_msgs::CoaxState> findClosestStampedState(roslib::Header header);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sequence_poses");
    ros::NodeHandle n("sequence_poses");
    
    getParams(n);
    
    ros::Subscriber filtered_blobs_sub = n.subscribe("/blob_pattern_finder/sequences", SEQ_MSG_BUFFER, &blobSequencesCallback);
    ros::Subscriber coax_state_sub = n.subscribe("/coax_server/state",STATE_MSG_BUFFER, &stateCallback);
    seq_pose_pub = n.advertise<coax_client::BlobSequencePoses>("/sequence_poses/sequence_poses", MSG_QUEUE);
    
    ros::Rate loop_rate(PUBLISH_FREQ);
    
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

void blobSequencesCallback(coax_client::BlobSequences msg)
{
    static float center_x = msg.image_width/2.0;
    static float center_y = msg.image_height/2.0;
    static float degrees_per_pixel_vert = FIELD_OF_VIEW_VERT/msg.image_height;
    static float degrees_per_pixel_horiz = FIELD_OF_VIEW_HORIZ/msg.image_width;
    
    coax_client::BlobSequencePoses seq_poses;
    //boost::shared_ptr<coax_msgs::CoaxState> state = findClosestStampedState(msg.header);
    seq_poses.header = msg.header;
    
    float angle_horiz;
    float angle_vert;
    
    for(int i = msg.sequences.size()-1; i >= 0;i--)
    {
        coax_client::BlobSequencePose seq_pose;
        seq_pose.sequence = msg.sequences[i];
        angle_horiz = degrees_per_pixel_horiz*(msg.sequences[i].x-center_x);
        angle_vert  = degrees_per_pixel_vert*(msg.sequences[i].y-center_y);
        seq_pose.pose.y = 78.74*sin(angle_horiz*3.14159/180.0)*-1;//-state->roll);
        seq_pose.pose.x = 78.74*sin(angle_vert*3.14159/180.0)*-1;//-state->pitch)*-1;
        seq_pose.pose.theta = 0;//state->zfiltered;
        seq_poses.sequence_poses.push_back(seq_pose);
    }
    
    seq_pose_pub.publish(seq_poses);
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
            ROS_WARN("%s/publish_freq must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POSE_NODE_PUBLISH_FREQ);
      else
          ROS_WARN("No value set for %s/publish_freq. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POSE_NODE_PUBLISH_FREQ);
      PUBLISH_FREQ = DEFAULT_BLOB_POSE_NODE_PUBLISH_FREQ;
    }

    //number of blob sequences from blob_filter this node will buffer before it begins to drop them
    if (nh.getParam("sequence_msg_buffer", SEQ_MSG_BUFFER))
    {
        ROS_INFO("Set %s/sequence_msg_buffer to %d",nh.getNamespace().c_str(), SEQ_MSG_BUFFER);
    }
    else
    {
        if(nh.hasParam("sequence_msg_buffer"))
            ROS_WARN("%s/sequence_msg_buffer must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POSE_NODE_SEQ_MSG_BUFFER);
      else
          ROS_WARN("No value set for %s/sequences_msg_buffer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POSE_NODE_SEQ_MSG_BUFFER);
      SEQ_MSG_BUFFER = DEFAULT_BLOB_POSE_NODE_SEQ_MSG_BUFFER;
    }
    
    //number of states from coax_server this node will buffer before it begins to drop them
    if (nh.getParam("state_msg_buffer", STATE_MSG_BUFFER))
    {
        ROS_INFO("Set %s/state_msg_buffer to %d",nh.getNamespace().c_str(), STATE_MSG_BUFFER);
    }
    else
    {
        if(nh.hasParam("state_msg_buffer"))
            ROS_WARN("%s/state_msg_buffer must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POSE_NODE_STATE_MSG_BUFFER);
      else
          ROS_WARN("No value set for %s/state_msg_buffer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POSE_NODE_STATE_MSG_BUFFER);
      STATE_MSG_BUFFER = DEFAULT_BLOB_POSE_NODE_STATE_MSG_BUFFER;
    }
    
    //number of messages this node will queue for publishing before it drops data
    if (nh.getParam("msg_queue", MSG_QUEUE))
    {
        ROS_INFO("Set %s/msg_queue to %d",nh.getNamespace().c_str(), MSG_QUEUE);
    }
    else
    {
        if(nh.hasParam("msg_queue"))
          ROS_WARN("%s/msg_queue must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POSE_NODE_MSG_QUEUE);
      else
        ROS_WARN("No value set for %s/msg_queue. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_POSE_NODE_MSG_QUEUE);
      MSG_QUEUE = DEFAULT_BLOB_POSE_NODE_MSG_QUEUE;
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
