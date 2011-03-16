/**
 *  @file   BlobMapperNode.cpp
 *  @author Aaron Parker
 *  @date   03-10-2011
 *  @brief      
 */

#include <ros/ros.h>
//#include <coax_client/CoaxClientConst.h>
#include <coax_client/IDGridCells.h>
#include <nav_msgs/GridCells.h>

ros::Publisher grid_cells_pub;

using namespace std;

void knownGlobalCallback(const coax_client::IDGridCells::ConstPtr& msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visuals");
    ros::NodeHandle n("visuals");
    
    ros::Subscriber seq_pose_sub = n.subscribe("/blob_mapper/known_global_cells", 1, &knownGlobalCallback);
    grid_cells_pub = n.advertise<nav_msgs::GridCells>("/grid_cells", 1);
    
    ros::spin();
}

void knownGlobalCallback(const coax_client::IDGridCells::ConstPtr& msg)
{
    nav_msgs::GridCells new_msg;
    
    new_msg.header.stamp = msg->header.stamp;
    new_msg.header.frame_id = "ground_robot";
    new_msg.cell_width = msg->cell_width;
    new_msg.cell_height = msg->cell_height;
    new_msg.cells = msg->cells;
    
    grid_cells_pub.publish(new_msg);
}
