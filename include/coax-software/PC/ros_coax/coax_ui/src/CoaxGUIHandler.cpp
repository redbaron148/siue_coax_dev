// Just for constants, and their text conversion
#include <com/sbapi.h>

#include "coax_msgs/CoaxConfigureOAMode.h"
#include "coax_msgs/CoaxConfigureComm.h"
#include "coax_msgs/CoaxConfigureControl.h"
#include "coax_msgs/CoaxReachNavState.h"
#include "coax_msgs/CoaxSetTimeout.h"
#include "coax_msgs/CoaxControl.h"
#include "CoaxGUIHandler.h"

CoaxGUIHandler::CoaxGUIHandler(ros::NodeHandle & n) : QMainWindow(), controlTimer(this), guiTimer(this), spinner(1) {
	configureCoaX = false;
	manualControl = false;
	desiredPitch = 0;
	desiredRoll = 0;
	desiredYaw = 0;
	desiredAlt = 0;

	gui.setupUi(this);
	connect(gui.configureButton,SIGNAL(pressed()),this,SLOT(prepareConfiguration()));
	connect(gui.applyCancelBox,SIGNAL(accepted()),this,SLOT(applyConfiguration()));
	connect(gui.applyCancelBox,SIGNAL(rejected()),this,SLOT(cancelConfiguration()));
	connect(gui.keyboardControl,SIGNAL(toggled(bool)),this,SLOT(toggleKeyControl(bool)));
	connect(&controlTimer, SIGNAL(timeout()), this, SLOT(updateControl()));
	connect(&guiTimer, SIGNAL(timeout()), this, SLOT(updateGui()));

	cfgControlClt = n.serviceClient<coax_msgs::CoaxConfigureControl>("coax_server/configure_control");
	cfgCommClt = n.serviceClient<coax_msgs::CoaxConfigureComm>("coax_server/configure_comm");
	cfgOAClt = n.serviceClient<coax_msgs::CoaxConfigureOAMode>("coax_server/configure_oamode");
	reachNavStateClt = n.serviceClient<coax_msgs::CoaxReachNavState>("coax_server/reach_nav_state");
	setTimeoutClt = n.serviceClient<coax_msgs::CoaxSetTimeout>("coax_server/set_timeout");

	// Subscribe to the state
	ros::TransportHints hints;
	state_sub = n.subscribe("/coax_server/state",5,&CoaxGUIHandler::stateCallback, this, hints.udp());
	// Publishing the control
	control_pub = n.advertise<coax_msgs::CoaxControl>("coax_server/control",10);

	std::string transport = "theora";
    image_transport::ImageTransport it(n);
    camera_sub = it.subscribe("/cv_capture", 1, &CoaxGUIHandler::imageCallback, this, transport);

	spinner.start();
	guiTimer.start(100);

}

CoaxGUIHandler::~CoaxGUIHandler() {
	coax_msgs::CoaxReachNavState srv;
	srv.request.desiredState = SB_NAV_STOP;
	srv.request.timeout = 30.0;
	if (reachNavStateClt.call(srv))
	{
		ROS_INFO("Reach nav state: %ld", (long int)srv.response.result);
	}
	else
	{
		ROS_ERROR("Failed to call service reach nav state");
	}
}

#define R2D(x) ((x)*180./M_PI)

void CoaxGUIHandler::updateGui() {
	gui.rollDial->setValue(R2D(state.roll)); 
	gui.rollLCD->display(R2D(state.roll)); 
	gui.pitchDial->setValue(R2D(state.pitch)); 
	gui.pitchLCD->display(R2D(state.pitch)); 
	gui.yawDial->setValue(R2D(state.yaw)); 
	gui.yawLCD->display(R2D(state.yaw)); 
	gui.rollGyroLCD->display(R2D(state.gyro[0]));
	gui.pitchGyroLCD->display(R2D(state.gyro[1]));
	gui.yawGyroLCD->display(R2D(state.gyro[2]));
	gui.accelXLCD->display((state.accel[0]));
	gui.accelYLCD->display((state.accel[1]));
	gui.accelZLCD->display((state.accel[2]));
	gui.magnetoXLCD->display((state.magneto[0]));
	gui.magnetoYLCD->display((state.magneto[1]));
	gui.magnetoZLCD->display((state.magneto[2]));
	gui.temperatureLCD->display(state.imutemp);
	gui.altRawLCD->display(state.zrange);
	gui.altFiltLCD->display(state.zfiltered);
	gui.pressureLCD->display(state.pressure);
	gui.frontRangeSlider->setValue(100*state.hranges[SB_RANGE_FRONT]);
	gui.rightRangeSlider->setValue(100*state.hranges[SB_RANGE_RIGHT]);
	gui.leftRangeSlider->setValue(100*state.hranges[SB_RANGE_LEFT]);
	gui.backRangeSlider->setValue(100*state.hranges[SB_RANGE_BACK]);
	float battPercent = 100*(state.battery - 9.0)/(12.6 - 9.0);
	if (battPercent>100) battPercent = 100;
	if (battPercent<0) battPercent = 0;
	gui.batteryProgress->setValue(battPercent);
	gui.batteryLCD->display(state.battery);
	gui.rcThrottleLCD->display(state.rcChannel[SB_RC_THROTTLE]);
	gui.rcThrottleTrimLCD->display(state.rcChannel[SB_RC_THROTTLE_TRIM]);
	gui.rcYawLCD->display(state.rcChannel[SB_RC_YAW]);
	gui.rcYawTrimLCD->display(state.rcChannel[SB_RC_YAW_TRIM]);
	gui.rcRollLCD->display(state.rcChannel[SB_RC_ROLL]);
	gui.rcRollTrimLCD->display(state.rcChannel[SB_RC_ROLL_TRIM]);
	gui.rcPitchLCD->display(state.rcChannel[SB_RC_PITCH]);
	gui.rcPitchTrimLCD->display(state.rcChannel[SB_RC_PITCH_TRIM]);
	gui.servoRollLCD->display(state.o_attitude[0]);
	gui.servoPitchLCD->display(state.o_attitude[1]);
	gui.motorUpLCD->display(state.o_attitude[2]);
	gui.motorDownLCD->display(state.o_altitude);
	gui.desiredRollLCD->display(desiredRoll);
	gui.desiredPitchLCD->display(desiredPitch);
	gui.desiredYawLCD->display(desiredYaw);
	gui.desiredAltitudeLCD->display(desiredAlt);
	gui.cameraLabel->setPixmap(QPixmap::fromImage(video));
	if (!configureCoaX) {
		gui.navigationCombo->clear();
		gui.navigationCombo->insertItem(0,sbNavModeString(state.mode.navigation));
		gui.commModeCombo->clear();
		gui.commModeCombo->insertItem(0,sbCommModeString(state.mode.communication));
		gui.oavoidCombo->clear();
		gui.oavoidCombo->insertItem(0,sbOAModeString(state.mode.oavoid));
		gui.rollModeCombo->clear();
		gui.rollModeCombo->insertItem(0,sbCtrlModeString(state.mode.rollAxis));
		gui.pitchModeCombo->clear();
		gui.pitchModeCombo->insertItem(0,sbCtrlModeString(state.mode.pitchAxis));
		gui.yawModeCombo->clear();
		gui.yawModeCombo->insertItem(0,sbCtrlModeString(state.mode.yawAxis));
		gui.altModeCombo->clear();
		gui.altModeCombo->insertItem(0,sbCtrlModeString(state.mode.altAxis));
		gui.watchdogTimeoutSpin->setValue(state.watchdogTimeout);
		gui.ctrlTimeoutSpin->setValue(state.controlTimeout);
	}
}


const static int navmodes[] = {SB_NAV_STOP,SB_NAV_IDLE,SB_NAV_HOVER,
	SB_NAV_CTRLLED, SB_NAV_RAW, -1};
const static int commmodes[] = {SB_COM_CONTINUOUS, SB_COM_ONREQUEST, -1};
const static int oamodes[] = {SB_OA_NONE, SB_OA_VERTICAL, SB_OA_HORIZONTAL, 
	SB_OA_HORIZONTAL | SB_OA_VERTICAL, -1};
const static int rollmodes[] = {SB_CTRL_NONE, SB_CTRL_POS, SB_CTRL_MANUAL, -1};
const static int pitchmodes[] = {SB_CTRL_NONE, SB_CTRL_POS, SB_CTRL_MANUAL, -1};
const static int yawmodes[] = {SB_CTRL_NONE, SB_CTRL_VEL, SB_CTRL_MANUAL, -1};
const static int altmodes[] = { SB_CTRL_NONE, SB_CTRL_REL, SB_CTRL_MANUAL | SB_CTRL_REL,
	SB_CTRL_POS, SB_CTRL_MANUAL | SB_CTRL_POS,
	SB_CTRL_FORCE, SB_CTRL_MANUAL | SB_CTRL_FORCE, -1};

static void insertModes(QComboBox * box, const int *modes,
		const char* (*textfunc)(unsigned char), unsigned int current)
{
	unsigned int i=0;
	int selected = -1;
	box->clear();
	while (modes[i]>=0) {
		box->insertItem(i,textfunc(modes[i]));
		if ((signed)current == modes[i]) selected = i;
		i++;
	}
	box->setCurrentIndex(selected);
}


void CoaxGUIHandler::prepareConfiguration()
{
	cfgstate = state;
	configureCoaX = true;
	gui.configureButton->setEnabled(false);
	gui.navigationCombo->clear();
	insertModes(gui.navigationCombo,navmodes,sbNavModeString,cfgstate.mode.navigation);

	insertModes(gui.commModeCombo,commmodes,sbCommModeString,cfgstate.mode.communication);

	insertModes(gui.oavoidCombo,oamodes,sbOAModeString,cfgstate.mode.oavoid);

	insertModes(gui.rollModeCombo,rollmodes,sbCtrlModeString,cfgstate.mode.rollAxis);
	insertModes(gui.pitchModeCombo,pitchmodes,sbCtrlModeString,cfgstate.mode.pitchAxis);
	insertModes(gui.yawModeCombo,yawmodes,sbCtrlModeString,cfgstate.mode.yawAxis);

	insertModes(gui.altModeCombo,altmodes,sbCtrlModeString,cfgstate.mode.altAxis);
	gui.applyCancelBox->setEnabled(true);
}

void CoaxGUIHandler::cancelConfiguration()
{
	configureCoaX = false;
	gui.configureButton->setEnabled(true);
	gui.applyCancelBox->setEnabled(false);
}

void CoaxGUIHandler::applyConfiguration()
{
	unsigned int selmode;
	gui.applyCancelBox->setEnabled(false);

	unsigned int ctrlTimeout = gui.ctrlTimeoutSpin->value();
	if (ctrlTimeout == 0) ctrlTimeout = 0xFFFF;
	unsigned int wdTimeout = gui.watchdogTimeoutSpin->value();
	if (wdTimeout == 0) wdTimeout = 0xFFFF;
	if ((wdTimeout != cfgstate.watchdogTimeout) || (ctrlTimeout != cfgstate.controlTimeout)) {
		ROS_INFO("Updating timeouts");
		coax_msgs::CoaxSetTimeout srv;
		srv.request.control_timeout_ms = ctrlTimeout;
		srv.request.watchdog_timeout_ms = wdTimeout;
		if (setTimeoutClt.call(srv))
		{
			ROS_INFO("Set timeout: %ld", (long int)srv.response.result);
		}
		else
		{
			ROS_ERROR("Failed to call service set timeout");
		}
	}


	selmode=navmodes[gui.navigationCombo->currentIndex()];
	if (selmode != cfgstate.mode.navigation) {
		ROS_INFO("Updating navigation");
		coax_msgs::CoaxReachNavState srv;
		srv.request.desiredState = selmode;
		srv.request.timeout = 30.0;
		if (reachNavStateClt.call(srv))
		{
			ROS_INFO("Reach nav state: %ld", (long int)srv.response.result);
		}
		else
		{
			ROS_ERROR("Failed to call service reach nav state");
		}
	}

	selmode=commmodes[gui.commModeCombo->currentIndex()];
	if (selmode != cfgstate.mode.communication) {
		ROS_INFO("Updating communication");
		coax_msgs::CoaxConfigureComm srv;
		srv.request.commMode = selmode;
		srv.request.frequency = 50;
		srv.request.numMessages = 1000;
		srv.request.contents = SBS_ALL;
		if (cfgCommClt.call(srv))
		{
			ROS_INFO("Configure comm: %ld", (long int)srv.response.result);
		}
		else
		{
			ROS_ERROR("Failed to call service configure comm");
		}
	}

	selmode=oamodes[gui.oavoidCombo->currentIndex()];
	if (selmode != cfgstate.mode.oavoid) {
		ROS_INFO("Updating obst. avoidance");
		coax_msgs::CoaxConfigureOAMode srv;
		srv.request.oavoidMode = selmode;
		if (cfgOAClt.call(srv))
		{
			ROS_INFO("Configure obst. avoid: %ld", (long int)srv.response.result);
		}
		else
		{
			ROS_ERROR("Failed to call service configure obst. avoid");
		}
	}

	unsigned int selroll, selpitch, selyaw, selalt;
	selroll=rollmodes[gui.rollModeCombo->currentIndex()];
	selpitch=pitchmodes[gui.pitchModeCombo->currentIndex()];
	selyaw=yawmodes[gui.yawModeCombo->currentIndex()];
	selalt=altmodes[gui.altModeCombo->currentIndex()];
	if ((selroll != cfgstate.mode.rollAxis) ||
			(selpitch != cfgstate.mode.pitchAxis) ||
			(selyaw != cfgstate.mode.yawAxis) ||
			(selalt != cfgstate.mode.altAxis)) {
		ROS_INFO("Updating control");
		coax_msgs::CoaxConfigureControl srv;
		srv.request.rollMode = selroll;
		srv.request.pitchMode = selpitch;
		srv.request.yawMode = selyaw;
		srv.request.altitudeMode = selalt;
		if (cfgControlClt.call(srv))
		{
			ROS_INFO("Configure control: %ld", (long int)srv.response.result);
		}
		else
		{
			ROS_ERROR("Failed to call service configure control");
		}
	}

	configureCoaX = false;
	gui.configureButton->setEnabled(true);
}

void CoaxGUIHandler::toggleKeyControl(bool ctrlstate) {
	manualControl = ctrlstate;
	ROS_INFO("Updating control");
	desiredPitch = 0;
	desiredRoll = 0;
	desiredYaw = 0;
	desiredAlt = state.zrange;

	if (manualControl) {
		coax_msgs::CoaxConfigureControl srv;
		srv.request.rollMode = SB_CTRL_POS;
		srv.request.pitchMode = SB_CTRL_POS;
		srv.request.yawMode = SB_CTRL_VEL;
		srv.request.altitudeMode = SB_CTRL_REL;
		if (cfgControlClt.call(srv))
		{
			ROS_INFO("Configure control: %ld", (long int)srv.response.result);
			controlTimer.start(50);
		}
		else
		{
			ROS_ERROR("Failed to call service configure control");
		}
	} else {
		controlTimer.stop();
	}

}

void CoaxGUIHandler::keyPressEvent(QKeyEvent *qkeyevent)
{
	if (!manualControl) {
		qkeyevent->ignore();
		desiredPitch = 0;
		desiredRoll = 0;
		desiredYaw = 0;
		desiredAlt = state.zrange;
		if (desiredAlt < 5e-2) desiredAlt = 0;
	} else {
		switch (qkeyevent->key())
		{
			case Qt::Key_PageUp:
				desiredAlt += 0.05;
				if (desiredAlt > 2.0) desiredAlt = 2.0;
				break;
			case Qt::Key_PageDown:
				desiredAlt -= 0.05;
				if (desiredAlt < 5e-2) desiredAlt = 0;
				break;
			case Qt::Key_Left:
				desiredRoll -= 0.05;
				if (desiredRoll < -1) desiredRoll = -1;
				break;
			case Qt::Key_Right:
				desiredRoll += 0.05;
				if (desiredRoll > 1) desiredRoll = 1;
				break;
			case Qt::Key_Up:
				desiredPitch -= 0.05;
				if (desiredPitch < -1) desiredPitch = -1;
				break;
			case Qt::Key_Down:
				desiredPitch += 0.05;
				if (desiredPitch > 1) desiredPitch = 1;
				break;
			case Qt::Key_S:
				desiredYaw += 0.05;
				if (desiredYaw > 1) desiredYaw = 1;
				break;
			case Qt::Key_D:
				desiredYaw -= 0.05;
				if (desiredYaw < -1) desiredYaw = -1;
				break;
			case Qt::Key_0:
				desiredPitch = 0;
				desiredRoll = 0;
				desiredYaw = 0;
			default:
				qkeyevent->ignore();
		}
	}
}

void CoaxGUIHandler::updateControl()
{
	if (state.mode.navigation == SB_NAV_CTRLLED) {
		coax_msgs::CoaxControl control;
		control.roll = desiredRoll;
		control.pitch = desiredPitch;
		control.yaw = desiredYaw;
		control.altitude = desiredAlt;
		control_pub.publish(control);
	}
}


