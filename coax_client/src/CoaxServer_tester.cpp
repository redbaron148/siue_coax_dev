/*
 *  File Name:      CoaxServerNode.cpp
 *  Programmer:     Aaron Parker
 *  Date Made:      01-11-2010
 *  Description:    ROS node, can be used to call service function calls to be 
 *                  performed by the COAX onboard gumstix.
 *				    
 */

#include <ros/ros.h>
#include <CoaxClientConst.h>
#include <coax_client/FindBlobPosition.h>
#include <coax_msgs/CoaxRequestState.h>

ros::ServiceClient find_blob_client;
ros::ServiceClient request_state_client;

void chatterCallback(const cmvision::Blobs &msg)
{
    coax_msgs::CoaxRequestState state_request;
    coax_client::FindBlobPosition find_blob_request;
    //coax_msgs::CoaxState state;
    
    state_request.request.contents = 0;
    
    if (request_state_client.call(state_request))
    {
        ROS_INFO("success!");
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }
  
    find_blob_request.request.blobs = msg;
    find_blob_request.request.blob_num = 0;
    find_blob_request.request.state = state_request.response.state;

    if (find_blob_client.call(find_blob_request))
    {
        ROS_INFO("another success!");
    }
    else
    {
        ROS_ERROR("Failed to call service");
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_blob_position_test");

  ros::NodeHandle n;
  find_blob_client = n.serviceClient<coax_client::FindBlobPosition>("/coax_services/add_two_ints");
  request_state_client = n.serviceClient<coax_msgs::CoaxRequestState>("/coax_server/request_state");
  
  ros::Subscriber sub = n.subscribe("/blobs", 1, chatterCallback);
  
  ros::spin();

  return 0;
}
