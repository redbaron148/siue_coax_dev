#ifndef COAX_GUI_HANDLER_H
#define COAX_GUI_HANDLER_H


#include <QObject>
#include <QDesktopWidget>
#include <QApplication>
#include <QtGui>
#include <QtGui/QMainWindow>

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/CvBridge.h>
#include <image_transport/image_transport.h>
#include "coax_msgs/CoaxState.h"
#include "ui_CoaxGUI.h"

class CoaxGUIHandler : public QMainWindow {
		Q_OBJECT
	protected:
		Ui_CoaxGUI gui;
		QImage video;
		QTimer controlTimer, guiTimer;
		ros::AsyncSpinner spinner;

		coax_msgs::CoaxState state,cfgstate;
		ros::Subscriber state_sub;
		ros::Publisher control_pub;
		ros::ServiceClient cfgControlClt;
		ros::ServiceClient cfgCommClt;
		ros::ServiceClient cfgOAClt;
		ros::ServiceClient reachNavStateClt;
		ros::ServiceClient setTimeoutClt;
		image_transport::Subscriber camera_sub;

		sensor_msgs::CvBridge img_bridge;
		boost::mutex image_mutex;



		bool configureCoaX;
		bool manualControl;
		float desiredRoll, desiredPitch, desiredYaw, desiredAlt;

	public:
		CoaxGUIHandler(ros::NodeHandle & n);
		~CoaxGUIHandler();

		void stateCallback(const coax_msgs::CoaxState::ConstPtr& msg) {
			// printf("Got State\n");
			state = *msg;
		}

		void imageCallback(const sensor_msgs::ImageConstPtr& msg)
		{
			boost::lock_guard<boost::mutex> guard(image_mutex);
			if (img_bridge.fromImage(*msg, "rgb8")) {
				// cvShowImage(window_name_.c_str(), img_bridge_.toIpl());
				IplImage *ipl = img_bridge.toIpl();
				video = QImage((const uchar*)(ipl->imageData),ipl->width,ipl->height,ipl->widthStep,QImage::Format_RGB888);
			} else {
				ROS_ERROR("Unable to convert %s image to bgr8", msg->encoding.c_str());
			}
		}

		/**
		 * Examine the key pressed and move the disc accordingly.
		 **/
		void keyPressEvent (QKeyEvent *);

	public slots:
		void updateGui(); 
		void updateControl();

		void prepareConfiguration();
		void cancelConfiguration();
		void applyConfiguration();

		void toggleKeyControl(bool state);
};


#endif // COAX_GUI_HANDLER_H
