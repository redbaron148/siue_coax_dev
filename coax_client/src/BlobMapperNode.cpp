/*
 *  File Name:      BlobMapperNode.cpp
 *  Programmer:     Aaron Parker
 *  Date Made:      03-10-2011
 *  Description:    
 */

#include <ros/ros.h>
#include <CoaxClientConst.h>
#include <coax_client/BlobSequencePoses.h>
#include <coax_client/IDGridCells.h>

//global variables
int PUBLISH_FREQ;
int SEQ_POSES_MSG_BUFFER;
int MSG_QUEUE;

coax_client::IDGridCells known_cells_global;
//ros::Publisher local_grid_cell_pub;

using namespace std;

void blobSequencePosesCallback(const coax_client::BlobSequencePoses::ConstPtr& msg);
void getParams(const ros::NodeHandle &nh);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_mapper");
    ros::NodeHandle n("blob_mapper");
    
    getParams(n);
    
    ros::Subscriber seq_pose_sub = n.subscribe("/blob_positions/sequence_poses", SEQ_POSES_MSG_BUFFER, &blobSequencePosesCallback);
    //local_grid_cell_pub = n.advertise<nav_msgs::GridCells>("/blob_mapper/local_grid_cells", 1);
    ros::Rate loop_rate(PUBLISH_FREQ);
    
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}

void blobSequencePosesCallback(const coax_client::BlobSequencePoses::ConstPtr& msg)
{
    coax_client::IDGridCells new_msg;
    new_msg.header.stamp = msg->header.stamp;
    new_msg.header.frame_id = "camera";
    new_msg.cell_width = .1;
    new_msg.cell_height = .1;
    
    for(int i = 0;i < msg->sequence_poses.size();i++)
    {
        geometry_msgs::Point point;
        if(msg->sequence_poses[i].sequence.id == GROUND_ROBOT_ID)
        {
            cout << "can see ground robot!" << endl;
            
        }
        point.x = msg->sequence_poses[i].pose.x/100.;
        point.y = msg->sequence_poses[i].pose.y/100.;
        new_msg.cells.push_back(point);
    }
    //local_grid_cell_pub.publish(new_msg);
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
            ROS_WARN("%s/publish_freq must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_MAPPER_NODE_PUBLISH_FREQ);
      else
          ROS_WARN("No value set for %s/publish_freq. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_MAPPER_NODE_PUBLISH_FREQ);
      PUBLISH_FREQ = DEFAULT_BLOB_MAPPER_NODE_PUBLISH_FREQ;
    }

    //number of blob sequences from blob_filter this node will buffer before it begins to drop them
    if (nh.getParam("sequence_poses_msg_buffer", SEQ_POSES_MSG_BUFFER))
    {
        ROS_INFO("Set %s/sequence_poses_msg_buffer to %d",nh.getNamespace().c_str(), SEQ_POSES_MSG_BUFFER);
    }
    else
    {
        if(nh.hasParam("sequences_msg_buffer"))
            ROS_WARN("%s/sequences_msg_buffer must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_MAPPER_NODE_SEQ_POSES_MSG_BUFFER);
      else
          ROS_WARN("No value set for %s/sequences_msg_buffer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_MAPPER_NODE_SEQ_POSES_MSG_BUFFER);
      SEQ_POSES_MSG_BUFFER = DEFAULT_BLOB_MAPPER_NODE_SEQ_POSES_MSG_BUFFER;
    }
    
    //number of messages this node will queue for publishing before it drops data
    if (nh.getParam("msg_queue", MSG_QUEUE))
    {
        ROS_INFO("Set %s/msg_queue to %d",nh.getNamespace().c_str(), MSG_QUEUE);
    }
    else
    {
        if(nh.hasParam("msg_queue"))
          ROS_WARN("%s/msg_queue must be an integer. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_MAPPER_NODE_MSG_QUEUE);
      else
        ROS_WARN("No value set for %s/msg_queue. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_BLOB_MAPPER_NODE_MSG_QUEUE);
      MSG_QUEUE = DEFAULT_BLOB_MAPPER_NODE_MSG_QUEUE;
    }
}
