/**
 * @author Nate Roney
 * copyright 2011
 *
     This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 *
 * gumstix_cam_node
 *
 * NOTE: this node was created to work with a custom version of OpenCV on a gumstix.
 *              It is not meant for desktop use.
 *
 * A ROS node whose function is to capture an image from a webcam, convert that Image
 *  from an OpenCV IplImage to a ROS Image Message, and to broadcast the
 *  converted image over the ROS network on the topic "image_<V4LID>", where V4LID matches
 *   the camera ID.
 *
 * Default behavior is to use the second camera attached to the computer. This is done
 *  because most of our laptops have an integrated webcam, and we typically prefer an external.
 *
 *  @params
 *
 *  v4l_id : the V4L index of the camera to be used. These correspond to /dev/video*
 *
 *  fps : framerate at which to capture. Default is 10 
 */

#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
//#include "opencv/highgui.h"
#include "sensor_msgs/fill_image.h"
#include <string>
#include <boost/lexical_cast.hpp>
#include "v4l_cam_node/libcam.h"

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gumstix_cam_node");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");
    
    string topic = "image_";
    string device = "/dev/video";

    int camID = 0;
    int FPS = 10;
    int height = 480;
    int width = 640;
    int hue = 0;
    int saturation = 0;
    int contrast = 0;
    int sharpness = 0;
    int brightness = 0;
    int white_balance_temp = 0;
    int exposure_auto = 0;
    int exposure_absolute = 0;
    bool auto_white_balance = true;
    
    nh.param( "v4l_id", camID, int(camID) );
    nh.param( "fps", FPS, int(FPS));
    nh.param( "height", height, int(height));
    nh.param( "width", width, int(width));
    
    ROS_INFO( "Using USB camera at V4L index: %d at %d FPS. Image is %d x %d.", camID, FPS , width, height);    
    //delete params to support multiple instances of this node with different
    // config options
    nh.deleteParam( "v4l_id" );
    nh.deleteParam( "fps" );    
    
    //append the V4L id to this topic for multiple camera support
    topic  += boost::lexical_cast<std::string>(camID);
    device += boost::lexical_cast<std::string>(camID);

    //setup the publisher
    ros::Publisher cameraPub = n.advertise<sensor_msgs::Image>(topic, 1);
    
    ros::Rate loop_rate(FPS);
    sensor_msgs::CvBridge bridge_;
    IplImage *rawImg = cvCreateImage(cvSize(width, height), 8, 3);
    int cap_status = 0;

    Camera cap_device( device.c_str(), width, height, FPS );
    
    nh.param( "brightness",brightness,int(cap_device.defaultBrightness()));
    nh.param( "contrast",contrast,int(cap_device.defaultContrast()));
    nh.param( "saturation",saturation,int(cap_device.defaultSaturation()));
    nh.param( "sharpness",sharpness,int(cap_device.defaultSharpness()));
    nh.param( "hue",hue,int(cap_device.defaultHue()));
    nh.param( "auto_white_balance",auto_white_balance,auto_white_balance);
    nh.param( "white_balance_temp",white_balance_temp,cap_device.defaultWhiteBalanceTemp());
    nh.param( "exposure_auto",exposure_auto,cap_device.defaultExposureAuto());
    nh.param( "exposure_absolute",exposure_absolute,cap_device.defaultExposureAbsolute());
    
    ROS_INFO("brightness: %d",brightness);
    ROS_INFO("contrast: %d",contrast);
    ROS_INFO("hue: %d",hue);
    ROS_INFO("saturation: %d",saturation);
    ROS_INFO("sharpness: %d",sharpness);
    ROS_INFO("auto white balance: %d",auto_white_balance);
    ROS_INFO("white balance temp: %d",white_balance_temp);
    ROS_INFO("exposure auto: %d",exposure_auto);
    ROS_INFO("exposure absolute: %d\n",exposure_absolute);

    cap_device.setHue(hue);
    cap_device.setBrightness(brightness);
    cap_device.setContrast(contrast);
    cap_device.setSaturation(saturation);
    cap_device.setSharpness(sharpness);
    cap_device.setAutoWhiteBalance(true);
    cap_device.setWhiteBalanceTemp(white_balance_temp);
    cap_device.setExposureAuto(exposure_auto);
    cap_device.setExposureAbsolute(exposure_absolute);
    sleep(1);
    cap_device.setAutoWhiteBalance(auto_white_balance);
    
    ROS_INFO("Min-Max brightness: %d-%d",cap_device.minBrightness(),cap_device.maxBrightness());
    ROS_INFO("Min-Max contrast: %d-%d",cap_device.minContrast(),cap_device.maxContrast());
    ROS_INFO("Min-Max saturation: %d-%d",cap_device.minSaturation(),cap_device.maxSaturation());
    ROS_INFO("Min-Max hue: %d-%d",cap_device.minHue(),cap_device.maxHue());
    ROS_INFO("Min-Max sharpness: %d-%d",cap_device.minSharpness(),cap_device.maxSharpness());
    ROS_INFO("Min-Max white balance temp: %d-%d",cap_device.minWhiteBalanceTemp(),cap_device.maxWhiteBalanceTemp());
    ROS_INFO("Min-Max exposure auto: %d-%d",cap_device.minExposureAuto(),cap_device.maxExposureAuto());
    ROS_INFO("Min-Max exposure absolute: %d-%d",cap_device.minExposureAbsolute(),cap_device.maxExposureAbsolute());

    while ( ros::ok() )
    {
        cap_device.Get() == 0 ? cap_status = 0 : cap_status = 1;    
        
        if( cap_status > 0 )
        {
            try
            {   
                //convert using cv_bridge and publish
                cap_device.toIplImage(rawImg);
                cameraPub.publish( bridge_.cvToImgMsg( rawImg ) );
            }
            catch ( sensor_msgs::CvBridgeException error )
            {
                ROS_ERROR( "error converting IplImage with CvBridge" );
            }
        }
        else
        {
            ROS_ERROR( "could not grab a frame from camera" );
        }
    
        loop_rate.sleep();
    }
    
    

    return 0;
}

