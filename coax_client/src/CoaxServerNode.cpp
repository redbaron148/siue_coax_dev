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

//global variables
double FIELD_OF_VIEW;

using namespace std;

double calculateBlobPosition(coax_client::FindBlobPosition::Request &req,
                             coax_client::FindBlobPosition::Response &res );
void getParams(const ros::NodeHandle &nh);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coax_services");
  ros::NodeHandle n("coax_services");

  ros::ServiceServer service = n.advertiseService("find_blob_position", calculateBlobPosition);
  ROS_INFO("coax_services started.");
  ros::spin();

  return 0;
}

double calculateBlobPosition(coax_client::FindBlobPosition::Request &req,
                             coax_client::FindBlobPosition::Response &res )
{
    
}

void getParams(const ros::NodeHandle &nh)
{
    //field of view of the camera onboard the gumstix of the COAX helicopter in degrees.
    if (nh.getParam("field_of_view", FIELD_OF_VIEW))
    {
        ROS_INFO("Set %s/field_of_view to %d",nh.getNamespace().c_str(), FIELD_OF_VIEW);
    }
    else
    {
        if(nh.hasParam("field_of_view"))
            ROS_WARN("%s/field_of_view must be an double. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FIELD_OF_VIEW;
        else
            ROS_WARN("No value set for %s/field_of_view. Setting default value: %d",nh.getNamespace().c_str(), DEFAULT_FIELD_OF_VIEW);
        FIELD_OF_VIEW = DEFAULT_FIELD_OF_VIEW;
    }
}
