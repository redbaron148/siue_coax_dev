#include "ros/ros.h"
#include "coax_msgs/CoaxGetVersion.h"
#include "coax_msgs/CoaxConfigureComm.h"
#include "coax_msgs/CoaxConfigureControl.h"
#include "coax_msgs/CoaxConfigureOAMode.h"
#include "coax_msgs/CoaxGetControlParameters.h"
#include "coax_msgs/CoaxSetControlParameters.h"
#include "coax_msgs/CoaxGetTrimMode.h"
#include "coax_msgs/CoaxSetTrimMode.h"
#include "coax_msgs/CoaxGetSensorList.h"
#include "coax_msgs/CoaxReachNavState.h"
#include "coax_msgs/CoaxRequestState.h"
#include "coax_msgs/CoaxSendString.h"
#include "coax_msgs/CoaxSetVerbose.h"
#include "coax_msgs/CoaxSetAckMode.h"
#include "coax_msgs/CoaxSetControl.h"
#include "coax_msgs/CoaxSetRawControl.h"
#include "coax_msgs/CoaxSetTimeout.h"
#include "coax_msgs/CoaxConstants.h"
#include "coax_msgs/CoaxReset.h"
#include "coax_msgs/CoaxControl.h"
#include "coax_msgs/CoaxRawControl.h"
#include "coax_msgs/CoaxSetLight.h"

#include <com/sbapi.h>
#include <com/sbsimple.h>

//#define DEBUG(c) ROS_INFO("Executing "#c)
//#define DEBUG(c) res=0;ROS_INFO("Executing "#c);c;ROS_INFO("Result %d",res)
#define DEBUG(c) res=0;c;if (res) ROS_INFO("Result of "#c": %d",res)
#define CRITICAL(c) res=0;c;if (res) {ROS_INFO("Result of "#c": %d",res); return res;}


class SBController
{
	protected:
		SBApiSimpleContext *simple;
		unsigned long sensorList;
		int res;
		ros::Publisher *coax_pub;

		static void rosStateExportCB(SBHeliState * state, void * ctxt)
		{
			SBController * that = (SBController*)ctxt;
			that->rosStateExport(state);
		}
		
		void rosStateExport(SBHeliState * state) {
			if (!coax_pub) return;
			// TODO: create ROS message and send it
			coax_msgs::CoaxState stateMsg;
#define COPY(V) stateMsg.V = state->V
			stateMsg.header.stamp = ros::Time::now();
			stateMsg.header.seq = simple->packetcount;
			stateMsg.header.frame_id = "continuous";
			COPY(errorFlags);
			COPY(content);
			COPY(timeStamp);
			COPY(controlTimeout);
			COPY(watchdogTimeout);
			COPY(mode.navigation);
			COPY(mode.communication);
			COPY(mode.oavoid);
			COPY(mode.rollAxis);
			COPY(mode.pitchAxis);
			COPY(mode.yawAxis);
			COPY(mode.altAxis);
			COPY(roll);
			COPY(pitch);
			COPY(yaw);
			for (unsigned int i=0;i<3;i++) {
				COPY(gyro[i]);
				COPY(accel[i]);
				COPY(magneto[i]);
				COPY(o_attitude[i]);
			}
			COPY(imutemp);
			COPY(zrange);
			COPY(zfiltered);
			COPY(pressure);
			COPY(battery);
			for (unsigned int i=0;i<4;i++) {
				COPY(hranges[i]);
			}
			for (unsigned int i=0;i<8;i++) {
				COPY(rcChannel[i]);
			}
			COPY(coaxspeed.state);
			COPY(coaxspeed.light);
			COPY(coaxspeed.vel_x);
			COPY(coaxspeed.vel_y);
			COPY(o_altitude);
			COPY(o_tol);
			for (unsigned int i=0;i<2;i++) {
				COPY(o_oavoid[i]);
				COPY(o_xy[i]);
			}
#undef COPY
			coax_pub->publish(stateMsg);
		}

	public:
		SBController(SBApiSimpleContext *s) 
			: simple(s), sensorList(0), res(0) 
		{
			coax_pub = NULL;
			sbSimpleDefaultContext(simple);
		}

		~SBController() {
		}

		void registerPublisher(ros::Publisher *pub) {
			coax_pub = pub;
		}

		int initialise(const std::string & devname) {
			res = 0;
			sbSimpleParseChannel(simple,devname.c_str(),NULL);
			simple->initNavState = SB_NAV_STOP;
			simple->cmdTimeout = 5000;
			simple->ctrlTimeout = 2000;
			simple->rollCtrlMode = SB_CTRL_POS;
			simple->pitchCtrlMode = SB_CTRL_POS;
			simple->altCtrlMode = SB_CTRL_REL;
			simple->yawCtrlMode = SB_CTRL_VEL;
			simple->sensorList = &sensorList;

			simple->commFreq = 30;
			// simple->commContent = SBS_TIMESTAMP | SBS_RPY | SBS_IMU_ALL | SBS_RANGES_ALL | 
            //     SBS_PRESSURE | SBS_BATTERY | SBS_O_XY | SBS_XY_REL | SBS_COAXSPEED;
			simple->stateFunc = rosStateExportCB;
			simple->userData = this;
			simple->oaMode = SB_OA_NONE;

			CRITICAL(res = sbSimpleInitialise(simple));
			sbLockCommunication(&simple->control);
			CRITICAL(res = sbConfigureAckMode(&simple->control,0));
			sbUnlockCommunication(&simple->control);
			ROS_INFO("Channel connected, continuing");

			return res;
		}

		int terminate() {
			res = 0;
			DEBUG(res = sbSimpleTerminate(simple));
			return res;
		}


		bool get_version(coax_msgs::CoaxGetVersion::Request  &req,
				coax_msgs::CoaxGetVersion::Response &res )
		{
			res.version.apiVersion = simple->remoteVersion.apiVersion;
			res.version.imuVersion = simple->remoteVersion.imuVersion;
			res.version.controllerVersion = simple->remoteVersion.controllerVersion;
			res.version.compileTime = simple->remoteVersion.compileTime;
			ROS_INFO("sending back response: [%d]", (int)res.version.apiVersion);
			return true;
		}

		bool configure_comm(coax_msgs::CoaxConfigureComm::Request  &req,
				coax_msgs::CoaxConfigureComm::Response &out )
		{
			DEBUG(res = sbSimpleConfigureComm(simple,
						req.frequency,req.contents & sensorList));
			ROS_INFO("Communication mode configured [%d] ", res);
			return true;
		}

		bool configure_control(coax_msgs::CoaxConfigureControl::Request  &req,
				coax_msgs::CoaxConfigureControl::Response &out )
		{
			simple->rollCtrlMode = req.rollMode;
			simple->pitchCtrlMode = req.pitchMode;
			simple->yawCtrlMode = req.yawMode;
			simple->altCtrlMode = req.altitudeMode;
			sbLockCommunication(&simple->control);
			DEBUG(res = sbConfigureControl(&simple->control,
						req.rollMode,req.pitchMode,req.yawMode,req.altitudeMode));
			sbUnlockCommunication(&simple->control);
			out.result = res;
			ROS_INFO("Configure control mode [%d]", res);
			return true;
		}

		bool configure_oamode(coax_msgs::CoaxConfigureOAMode::Request  &req,
				coax_msgs::CoaxConfigureOAMode::Response &out )
		{
			sbLockCommunication(&simple->control);
			DEBUG(res = sbConfigureOAMode(&simple->control,req.oavoidMode));
			sbUnlockCommunication(&simple->control);
			out.result = res;
			ROS_INFO("Configure obst. avoid mode [%d]", res);
			return true;
		}

		bool set_light(coax_msgs::CoaxSetLight::Request  &req,
				coax_msgs::CoaxSetLight::Response &out )
		{
			sbLockCommunication(&simple->control);
			DEBUG(res = sbSetCoaxSpeedLight(&simple->control,req.percent));
			sbUnlockCommunication(&simple->control);
			out.result = res;
			ROS_INFO("Coax set light %d [%d]", req.percent, res);
			return true;
		}

		bool get_control_parameters(coax_msgs::CoaxGetControlParameters::Request  &req,
				coax_msgs::CoaxGetControlParameters::Response &out )
		{
			struct SBControlParameters parms;
			sbLockCommunication(&simple->control);
			DEBUG(res = sbGetControlParameters(&simple->control,&parms));
			sbUnlockCommunication(&simple->control);
			out.params.baseThrust = parms.baseThrust;
			out.params.yawOffset = parms.yawOffset;
			out.params.altitudeKp = parms.altitudeKp;
			out.params.altitudeKi = parms.altitudeKi;
			out.params.altitudeKd = parms.altitudeKd;
			out.params.yawKp = parms.yawKp;
			out.params.yawKi = parms.yawKi;
			out.params.yawKd = parms.yawKd;
			ROS_INFO("Get control parameters [%d]", res);
			return true;
		}

		bool set_control_parameters(coax_msgs::CoaxSetControlParameters::Request  &req,
				coax_msgs::CoaxSetControlParameters::Response &out )
		{
			struct SBControlParameters parms;
			parms.baseThrust = req.params.baseThrust;
			parms.yawOffset = req.params.yawOffset;
			parms.altitudeKp = req.params.altitudeKp;
			parms.altitudeKi = req.params.altitudeKi;
			parms.altitudeKd = req.params.altitudeKd;
			parms.yawKp = req.params.yawKp;
			parms.yawKi = req.params.yawKi;
			parms.yawKd = req.params.yawKd;
			sbLockCommunication(&simple->control);
			DEBUG(res = sbSetControlParameters(&simple->control,&parms));
			sbUnlockCommunication(&simple->control);
			out.result = res;
			ROS_INFO("Set control parameters [%d]", res);
			return true;
		}

		bool get_trim_mode(coax_msgs::CoaxGetTrimMode::Request  &req,
				coax_msgs::CoaxGetTrimMode::Response &out )
		{
			struct SBTrimMode mode;
			sbLockCommunication(&simple->control);
			DEBUG(res = sbGetTrimMode(&simple->control,&mode));
			sbUnlockCommunication(&simple->control);
			out.mode.trimMode = mode.trimMode;
			out.mode.rollTrim = mode.rollTrim;
			out.mode.pitchTrim = mode.pitchTrim;
			ROS_INFO("Get trim mode [%d]", res);
			return true;
		}

		bool set_trim_mode(coax_msgs::CoaxSetTrimMode::Request  &req,
				coax_msgs::CoaxSetTrimMode::Response &out )
		{
			struct SBTrimMode mode;
			mode.trimMode = req.mode.trimMode;
			mode.rollTrim = req.mode.rollTrim;
			mode.pitchTrim = req.mode.pitchTrim;
			sbLockCommunication(&simple->control);
			DEBUG(res = sbSetTrimMode(&simple->control,&mode));
			sbUnlockCommunication(&simple->control);
			out.result = res;
			ROS_INFO("Set trim mode [%d]", out.result);
			return true;
		}

		bool get_sensor_list(coax_msgs::CoaxGetSensorList::Request  &req,
				coax_msgs::CoaxGetSensorList::Response &out )
		{
			out.list = sensorList;
			return true;
		}

		bool request_state(coax_msgs::CoaxRequestState::Request  &req,
				coax_msgs::CoaxRequestState::Response &out )
		{
			SBHeliState state;
			sbLockCommunication(&simple->control);
			DEBUG(res = sbRequestState(&simple->control,req.contents & sensorList,&state));
			sbUnlockCommunication(&simple->control);
			ROS_INFO("Request state [%d]", res);

#define COPY(V) out.state.V = state.V
			out.state.header.stamp = ros::Time::now();
			out.state.header.seq = 0;
			out.state.header.frame_id = "request";
			COPY(errorFlags);
			COPY(content);
			COPY(timeStamp);
			COPY(controlTimeout);
			COPY(watchdogTimeout);
			COPY(mode.navigation);
			COPY(mode.communication);
			COPY(mode.oavoid);
			COPY(mode.rollAxis);
			COPY(mode.pitchAxis);
			COPY(mode.yawAxis);
			COPY(mode.altAxis);
			COPY(roll);
			COPY(pitch);
			COPY(yaw);
			for (unsigned int i=0;i<3;i++) {
				COPY(gyro[i]);
				COPY(accel[i]);
				COPY(magneto[i]);
				COPY(o_attitude[i]);
			}
			COPY(imutemp);
			COPY(zrange);
			COPY(zfiltered);
			COPY(pressure);
			COPY(battery);
			for (unsigned int i=0;i<4;i++) {
				COPY(hranges[i]);
			}
			for (unsigned int i=0;i<8;i++) {
				COPY(rcChannel[i]);
			}
			COPY(o_altitude);
			COPY(o_tol);
			for (unsigned int i=0;i<2;i++) {
				COPY(o_oavoid[i]);
				COPY(o_xy[i]);
			}
#undef COPY

			return true;
		}

		bool reach_nav_state(coax_msgs::CoaxReachNavState::Request  &req,
				coax_msgs::CoaxReachNavState::Response &out )
		{
			ROS_INFO("Reaching nav state %s [%f]",sbNavModeString(req.desiredState),req.timeout);
			res = sbSimpleReachNavState(simple,req.desiredState,req.timeout);
			out.result = res;
			ROS_INFO("Reached nav state %s [%d]",sbNavModeString(req.desiredState),out.result);
			return true;
		}

		bool set_ack_mode(coax_msgs::CoaxSetAckMode::Request  &req,
				coax_msgs::CoaxSetAckMode::Response &out )
		{
			sbLockCommunication(&simple->control);
			DEBUG(res = sbConfigureAckMode(&simple->control,req.mode));
			sbUnlockCommunication(&simple->control);
			ROS_INFO("set ack mode [%d]", req.mode);
			out.result = res;
			return true;
		}

		bool set_timeout(coax_msgs::CoaxSetTimeout::Request  &req,
				coax_msgs::CoaxSetTimeout::Response &out )
		{
			sbLockCommunication(&simple->control);
			DEBUG(res = sbConfigureTimeout(&simple->control,
						req.control_timeout_ms,req.watchdog_timeout_ms));
			sbUnlockCommunication(&simple->control);
			ROS_INFO("set timeout [%04X %04X]", req.control_timeout_ms,req.watchdog_timeout_ms);
			out.result = res;
			return true;
		}

		bool send_string(coax_msgs::CoaxSendString::Request  &req,
				coax_msgs::CoaxSendString::Response &out )
		{
			sbLockCommunication(&simple->control);
			DEBUG(res = sbSendString(&simple->control,req.text.c_str()));
			sbUnlockCommunication(&simple->control);
			ROS_INFO("sending string [%s]", req.text.c_str());
			out.result = res;
			return true;
		}

		bool set_verbose(coax_msgs::CoaxSetVerbose::Request  &req,
				coax_msgs::CoaxSetVerbose::Response &out )
		{
			res = sbSimpleSetVerbose(simple,req.verbose,req.channel);
			out.result = res;
			ROS_INFO("Set verbosity [%d]", req.verbose);
			return true;
		}

		bool set_control(coax_msgs::CoaxSetControl::Request  &req,
				coax_msgs::CoaxSetControl::Response &out )
		{
			res = sbSimpleControl(simple,req.roll,req.pitch,
					req.yaw,req.altitude);
			out.result = res;
			ROS_INFO("Set Control [%d]", out.result);
			return true;
		}

		bool set_raw_control(coax_msgs::CoaxSetRawControl::Request  &req,
				coax_msgs::CoaxSetRawControl::Response &out )
		{
			res = sbSimpleRawControl(simple,req.motor1,req.motor2,
					req.servo1,req.servo2);
			out.result = res;
			ROS_INFO("Set Control [%d]", out.result);
			return true;
		}

		bool reset(coax_msgs::CoaxReset::Request  &req,
				coax_msgs::CoaxReset::Response &out )
		{
			std::string dev = simple->device;
			DEBUG(res = terminate());
			DEBUG(res = initialise(dev));
			out.result = res;
			ROS_INFO("Reset [%d]", out.result);
			return true;
		}

		void control_callback(const coax_msgs::CoaxControl::ConstPtr & message) {
			DEBUG(res = sbSimpleControl(simple,message->roll,message->pitch,message->yaw,message->altitude));
		}

		void raw_control_callback(const coax_msgs::CoaxRawControl::ConstPtr & message) {
			DEBUG(res = sbSimpleRawControl(simple,message->motor1,message->motor2,message->servo1,message->servo2));
		}
};

int main(int argc, char **argv)
{
	int res;
	const SBVersionStatus * compiled = sbGetCompiledVersion(); 
	unsigned int objSizeStatus = sbValidateObjectSize(compiled);

	ros::init(argc, argv, "coax_server");
	ROS_INFO("Object size status: %04x",objSizeStatus);
	assert(objSizeStatus == 0);

	SBApiSimpleContext simple;
	SBController api(&simple);


	CRITICAL(res = api.initialise((argc<2)?("localhost"):(argv[1])));

	ros::NodeHandle n("/coax_server");

	std::vector<ros::ServiceServer> services;
	services.push_back(n.advertiseService("get_version", &SBController::get_version, &api));
	services.push_back(n.advertiseService("configure_comm", &SBController::configure_comm, &api));
	services.push_back(n.advertiseService("configure_control", &SBController::configure_control, &api));
	services.push_back(n.advertiseService("configure_oamode", &SBController::configure_oamode, &api));
	services.push_back(n.advertiseService("get_control_parameters", &SBController::get_control_parameters, &api));
	services.push_back(n.advertiseService("set_control_parameters", &SBController::set_control_parameters, &api));
	services.push_back(n.advertiseService("get_trim_mode", &SBController::get_trim_mode, &api));
	services.push_back(n.advertiseService("set_trim_mode", &SBController::set_trim_mode, &api));
	services.push_back(n.advertiseService("get_sensor_list", &SBController::get_sensor_list, &api));
	services.push_back(n.advertiseService("reach_nav_state", &SBController::reach_nav_state, &api));
	services.push_back(n.advertiseService("request_state", &SBController::request_state, &api));
	services.push_back(n.advertiseService("send_string", &SBController::send_string, &api));
	services.push_back(n.advertiseService("set_verbose", &SBController::set_verbose, &api));
	services.push_back(n.advertiseService("set_ack_mode", &SBController::set_ack_mode, &api));
	services.push_back(n.advertiseService("set_timeout", &SBController::set_timeout, &api));
	services.push_back(n.advertiseService("set_control", &SBController::set_control, &api));
	services.push_back(n.advertiseService("set_raw_control", &SBController::set_raw_control, &api));
	services.push_back(n.advertiseService("reset", &SBController::reset, &api));
	services.push_back(n.advertiseService("set_light", &SBController::set_light, &api));

	// Publishing the state
	ros::Publisher coax_pub = n.advertise<coax_msgs::CoaxState>("state",50);
	api.registerPublisher(&coax_pub);

	// Subscribing to control message (without acknowledgement)
	ros::TransportHints hints;
	ros::Subscriber control_sub = n.subscribe("control",10,&SBController::control_callback,&api,hints.udp());
	ros::Subscriber rawcontrol_sub = n.subscribe("rawcontrol",10,&SBController::raw_control_callback,&api,hints.udp());

	ROS_INFO("Coax Server ready");
	ros::spin();

	DEBUG(res = api.terminate());
	return 0;
}

