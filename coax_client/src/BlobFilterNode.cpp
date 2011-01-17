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
#include <coax_client/FilteredBlobs.h>
#include <coax_msgs/CoaxState.h>

//global variables
double FIELD_OF_VIEW_HORIZ;
double FIELD_OF_VIEW_VERT;

//calculated variables
double ratio;

using namespace std;

ros::Publisher filtered_blob_pub;
boost::shared_ptr<coax_msgs::CoaxState> cur_state;

void blobsCallback(cmvision::Blobs msg);
void stateCallback(boost::shared_ptr<coax_msgs::CoaxState> msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blob_filter");
    ros::NodeHandle n("blob_filter");

    ros::Subscriber state_sub = n.subscribe("/coax_server/state", 1, &stateCallback);
    ros::Subscriber blobs_sub = n.subscribe("/blobs", 1, &blobsCallback);

    filtered_blob_pub = n.advertise<coax_client::FilteredBlobs>("/blob_filter/blobs", 1);

    ros::spin();

    return 0;
}

void blobsCallback(cmvision::Blobs msg)
{
    coax_client::FilteredBlobs fblobs;
    fblobs.header.stamp = ros::Time::now();
    fblobs.header.frame_id = "coax";
    fblobs.yaw = cur_state->yaw;
    fblobs.pitch = cur_state->pitch;
    fblobs.roll = cur_state->roll;
    fblobs.altitude = cur_state->zfiltered;
    fblobs.blobs = msg;
    
    filter_blob_pub.publish(fblobs);
}

void stateCallback(boost::shared_ptr<coax_msgs::CoaxState> msg)
{
    cur_state = msg;
}
