/**
 *  @file   BlobMapperNode.cpp
 *  @author Aaron Parker
 *  @date   03-10-2011
 *  @brief      
 */

#include <ros/ros.h>
#include <coax_client/CoaxClientConst.h>
#include <coax_client/BlobSequencePoses.h>
#include <coax_client/IDGridCells.h>
#include <geometry_msgs/PoseStamped.h>

//global variables
int PUBLISH_FREQ;
int SEQ_POSES_MSG_BUFFER;
int MSG_QUEUE;

coax_client::IDGridCells known_global_cells;
geometry_msgs::PoseStamped helicopter_pose;

ros::Publisher known_global_cells_pub;
ros::Publisher heli_pose_pub;

using namespace std;

void blobSequencePosesCallback(const coax_client::BlobSequencePoses::ConstPtr& msg);
void getParams(const ros::NodeHandle &nh);
int isInIDGridCells(const coax_client::IDGridCells& msg, const uint8_t &id);
int canSeeKnownCells(const coax_client::IDGridCells& msg);
void calculatePositionOfUnknown(const coax_client::IDGridCells& unknown,const coax_client::BlobSequencePoses::ConstPtr& msg,const int &ref);
void calculatePositionOfHelicopter(const coax_client::BlobSequencePoses::ConstPtr& msg, const int &ref)
{
    if(known_global_cells.referance == -1) return;
    //cout << "known referance: " << known_global_cells.referance << "  id: " << known_global_cells.ids[known_global_cells.referance] << endl;
    //cout << "local referance: " << ref << "  id: " << int(msg->sequence_poses[ref].sequence.id) << endl;
    
    helicopter_pose.header.stamp = msg->header.stamp;
    float local_x = -msg->sequence_poses[ref].pose.x;
    float local_y = -msg->sequence_poses[ref].pose.y;
    float global_x = known_global_cells.cells[known_global_cells.referance].x;
    float global_y = known_global_cells.cells[known_global_cells.referance].y;
    
    //cout << " local: (" << local_x << "," << local_y << ")" << endl;
    //cout << "global: (" << global_x << "," << global_y << ")" << endl;

    helicopter_pose.pose.position.x = local_x+global_x+CAMERA_X_OFFSET;
    helicopter_pose.pose.position.y = local_y+global_y+CAMERA_Y_OFFSET;
    helicopter_pose.pose.position.z = msg->altitude;
    
    //cout << "  heli: (" << helicopter_pose.pose.position.x << "," << helicopter_pose.pose.position.y << ")" << endl << endl;
    
    heli_pose_pub.publish(geometry_msgs::PoseStamped(helicopter_pose));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sequence_mapper");
    ros::NodeHandle n("sequence_mapper");
    
    getParams(n);
    
    known_global_cells.header.stamp = ros::Time::now();
    
    known_global_cells.cells.push_back(geometry_msgs::Point());
    known_global_cells.ids.push_back(GROUND_ROBOT_ID);
    known_global_cells.cell_width = .1;
    known_global_cells.cell_height = .1;
    helicopter_pose.header.frame_id = "ground_robot";
    
    //cout << known_global_cells;
    
    ros::Subscriber seq_pose_sub = n.subscribe("/sequence_poses/sequence_poses", SEQ_POSES_MSG_BUFFER, &blobSequencePosesCallback);
    known_global_cells_pub = n.advertise<coax_client::IDGridCells>("/blob_mapper/known_global_cells", 1);
    heli_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/blob_mapper/helicopter_pose",1);
    
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
    static bool found_ground_robot_flag;
    //static bool found_known_cell_flag;
    //static int ground_robot_array_pos;
    static int known_referance;
    known_referance = -1;
    
    //ground_robot_array_pos = -1;
    found_ground_robot_flag = false;
    
    coax_client::IDGridCells unknown;
    unknown.header.stamp = msg->header.stamp;
    unknown.header.frame_id = "camera";
    unknown.cell_width = .1;
    unknown.cell_height = .1;
    unknown.referance = -1;
    
    for(unsigned int i = 0;i < msg->sequence_poses.size();i++)
    {
        if(!found_ground_robot_flag && msg->sequence_poses[i].sequence.id == GROUND_ROBOT_ID)
        {   
            found_ground_robot_flag = true;
            unknown.referance = known_global_cells.referance = 0;
            known_referance = i;
        }
        
        //is the cell already in known_globals?
        int temp = isInIDGridCells(known_global_cells,msg->sequence_poses[i].sequence.id);
        
        //if the cell being viewed is NOT already being tracked in known_globals
        if(temp==-1)
        {
            geometry_msgs::Point point;
            point.x = msg->sequence_poses[i].pose.x;
            point.y = msg->sequence_poses[i].pose.y;
            unknown.cells.push_back(point);
            unknown.ids.push_back(msg->sequence_poses[i].sequence.id);
            //cout << "added a sequence to unkown with id of " << int(msg->sequence_poses[i].sequence.id) << endl;
        }
        //if the sequence being viewed IS already being tracked in known_globals
        else 
        {
            //if we have not found the ground robot, set this poit as the referance point for calculating other positions
            if(!found_ground_robot_flag)
            { 
                unknown.referance = known_global_cells.referance = temp;
                known_referance = i;
            }
            //if(!found_known_cell_flag) found_known_cell_flag = true;
        }
    }
    
    calculatePositionOfUnknown(unknown,msg,known_referance);
    calculatePositionOfHelicopter(msg,known_referance);
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

int isInIDGridCells(const coax_client::IDGridCells& msg, const uint8_t &id)
{
    int i;
    for(i = msg.ids.size()-1;i >= 0;i--)
    {
        //cout << "looking for " << int(id) << " in position " << i << endl;
        if(msg.ids[i]==id) break;
    }
    return i;
}

int canSeeKnownCells(const coax_client::IDGridCells& msg)
{
    int temp = -1;
    for(int i = msg.ids.size()-1;i>=0;i--)
    {
        temp = isInIDGridCells(known_global_cells,msg.ids[i]);
        if(temp!=-1) return temp;
    }
    return temp;
}

void calculatePositionOfUnknown(const coax_client::IDGridCells& unknown,const coax_client::BlobSequencePoses::ConstPtr& msg, const int &ref)
{
    //ROS_DEBUG("calculating position of unknown");
    if(unknown.referance == -1 || !unknown.ids.size()) 
    {
        //cout << "referance: " << msg.referance << "  size: " << msg.ids.size() << endl;
        return;
    }
    
    float known_local_x = msg->sequence_poses[ref].pose.x;
    float known_local_y = msg->sequence_poses[ref].pose.y;
    float known_global_x = known_global_cells.cells[known_global_cells.referance].x;
    float known_global_y = known_global_cells.cells[known_global_cells.referance].y;
    
    int i = 0;
    for(i=unknown.ids.size()-1;i>=0;i--)
    {
        geometry_msgs::Point point;
        
        float unknown_local_x = unknown.cells[i].x;
        float unknown_local_y = unknown.cells[i].y;
        
        point.x = -known_local_x+unknown_local_x+known_global_x;
        point.y = -known_local_y+unknown_local_y+known_global_y;
        
        //cout << "(" << int(msg->sequence_poses[ref].sequence.id) << ")   known: (" << known_local_x << "," << known_local_y << ")L" << endl;
        //cout << "(" << known_global_cells.ids[known_global_cells.referance] << ")   known: (" << known_global_x << "," << known_global_y << ")G" << endl;
        //cout << "(" << unknown.ids[i] << ") unknown: (" << unknown_local_x << "," << unknown_local_y << ")L" << endl;
        //cout << "(" << unknown.ids[i] << ") unknown: (" << point.x << "," << point.y << ")G" << endl;
        known_global_cells.cells.push_back(point);
        known_global_cells.ids.push_back(int(unknown.ids[i]));
    }
    //cout << "bwah!!!!!" << endl;
    known_global_cells_pub.publish(coax_client::IDGridCells(known_global_cells));
}
