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
double FIELD_OF_VIEW_HORIZ;
double FIELD_OF_VIEW_VERT;

//calculated variables
double ratio;

using namespace std;

bool calculateBlobPosition(coax_client::FindBlobPosition::Request &req,
                           coax_client::FindBlobPosition::Response &res );
void getParams(const ros::NodeHandle &nh);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coax_services");
  ros::NodeHandle n("coax_services");
  
  getParams(n);

  ros::ServiceServer service = n.advertiseService("find_blob_position", calculateBlobPosition);
  ROS_INFO("coax_services started.");
  ros::spin();

  return 0;
}

bool calculateBlobPosition(coax_client::FindBlobPosition::Request &req,
                             coax_client::FindBlobPosition::Response &res )
{
    float altitude = req.state.zfiltered;
    float x = (req.blobs.blobs[req.blob_num].right+req.blobs.blobs[req.blob_num].left)/2.0;
    float y = (req.blobs.blobs[req.blob_num].top+req.blobs.blobs[req.blob_num].bottom)/2.0;
    unsigned short int width = req.blobs.image_width;
    unsigned short int height = req.blobs.image_height;
    float center_x = width/2.0;
    float center_y = height/2.0;
    float x_from_center = x-center_x;
    float y_from_center = y-center_y;
    
    float degrees_per_pixel_horiz   = FIELD_OF_VIEW_HORIZ/width;
    float degrees_per_pixel_vert    = FIELD_OF_VIEW_VERT/height;
    
    ROS_INFO("\n\ndegree horiz: %f\ndegree vert: %f\n",degrees_per_pixel_horiz*x_from_center, degrees_per_pixel_vert*y_from_center);
    
    return true;
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
}
