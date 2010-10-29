#include <math.h>
#include <stdlib.h>

#include <QObject>
#include <QDesktopWidget>
#include <QApplication>
#include <QtGui>
#include <QtGui/QMainWindow>
#include <QTimer>


#include <ros/ros.h>
#include "coax_msgs/CoaxState.h"
#include "coax_msgs/CoaxControl.h"


#include "CoaxGUIHandler.h"
#define R2D(x) ((x)*180./M_PI)


int main(int argc, char *argv[])
{
	ros::init(argc, argv, "CoaxGUI");
	ros::NodeHandle n;

	QApplication app(argc, argv);  

	CoaxGUIHandler window(n);


	window.setWindowTitle("Coax GUI");
	window.show();

	return app.exec();
}

