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

#include <com/sbapi.h>
#include <com/sbsimple.h>

//#define DEBUG(c) ROS_INFO("Executing "#c)
//#define DEBUG(c) res=0;ROS_INFO("Executing "#c);c;ROS_INFO("Result %d",res)
#define DEBUG(c) res=0;c;if (res) ROS_INFO("Result of "#c": %d",res)
#define CRITICAL(c) res=0;c;if (res) {ROS_INFO("Result of "#c": %d",res); return res;}


class SBController
{
	protected:
		unsigned long sensorList;
		int res;
		ros::Publisher *coax_pub;
        coax_msgs::CoaxState stateMsg;

		static void rosStateExportCB(SBHeliState * state, void * ctxt)
		{
			SBController * that = (SBController*)ctxt;
			that->rosStateExport(state);
		}
		
		void rosStateExport(SBHeliState * state) {
			if (!coax_pub) return;
			coax_pub->publish(stateMsg);
		}

	public:
		SBController() 
			: sensorList(0), res(0) 
		{
			coax_pub = NULL;
		}

		~SBController() {
		}


        void timerCallback(const ros::TimerEvent& timer_event) {
            static unsigned int counter = 0;
#define COPY(V,X) stateMsg.V = X
			COPY(errorFlags,0);
			COPY(content,0xDEADBEEF);
			COPY(timeStamp,12345678);
			COPY(controlTimeout,0xFFFF);
			COPY(watchdogTimeout,0xFFFF);
			COPY(mode.navigation,SB_NAV_STOP);
			COPY(mode.communication,SB_COM_CONTINUOUS);
			COPY(mode.oavoid,SB_OA_NONE);
			COPY(mode.rollAxis,SB_CTRL_MANUAL);
			COPY(mode.pitchAxis,SB_CTRL_MANUAL);
			COPY(mode.yawAxis,SB_CTRL_MANUAL);
			COPY(mode.altAxis,SB_CTRL_MANUAL);
			COPY(roll,remainder(counter * M_PI/180.,2*M_PI));
			COPY(pitch,remainder(counter * 2*M_PI/180.,2*M_PI));
			COPY(yaw,remainder(counter * 3*M_PI/180.,2*M_PI));
			for (unsigned int i=0;i<3;i++) {
				COPY(gyro[i],-1+2*drand48());
				COPY(accel[i],-10+2*drand48());
				COPY(magneto[i],-1+2*drand48());
				COPY(o_attitude[i],-1+2*drand48());
			}
			COPY(imutemp,20+5*drand48());
			COPY(zrange,drand48());
			COPY(zfiltered,stateMsg.zrange);
			COPY(pressure,(1+drand48())*1000.);
			COPY(battery,10. + 2*drand48());
			for (unsigned int i=0;i<4;i++) {
				COPY(hranges[i],drand48());
			}
			for (unsigned int i=0;i<8;i++) {
				COPY(rcChannel[i],drand48());
			}
			COPY(o_altitude,drand48());
			COPY(o_tol,drand48());
			for (unsigned int i=0;i<2;i++) {
				COPY(o_oavoid[i],drand48());
				COPY(o_xy[i],drand48());
			}
#undef COPY
            counter += 1;
			coax_pub->publish(stateMsg);
        }

		void registerPublisher(ros::Publisher *pub) {
			coax_pub = pub;
		}

		int initialise(const std::string & devname) {
			res = 0;
			ROS_INFO("Channel connected, continuing");
			return res;
		}

		int terminate() {
			res = 0;
			return res;
		}


		bool get_version(coax_msgs::CoaxGetVersion::Request  &req,
				coax_msgs::CoaxGetVersion::Response &res )
		{
			res.version.apiVersion = 0;
			res.version.imuVersion = "dummy";
			res.version.controllerVersion = 0;
			res.version.compileTime = "never";
			ROS_INFO("sending back response: [%d]", (int)res.version.apiVersion);
			return true;
		}

		bool configure_comm(coax_msgs::CoaxConfigureComm::Request  &req,
				coax_msgs::CoaxConfigureComm::Response &out )
		{
			out.result = -1;
			ROS_INFO("Configure comm mode [%d]", res);
			return true;
		}

		bool configure_control(coax_msgs::CoaxConfigureControl::Request  &req,
				coax_msgs::CoaxConfigureControl::Response &out )
		{
			out.result = -1;
			ROS_INFO("Configure control mode [%d]", res);
			return true;
		}

		bool configure_oamode(coax_msgs::CoaxConfigureOAMode::Request  &req,
				coax_msgs::CoaxConfigureOAMode::Response &out )
		{
			out.result = -1;
			ROS_INFO("Configure obst. avoid mode [%d]", res);
			return true;
		}

		bool get_control_parameters(coax_msgs::CoaxGetControlParameters::Request  &req,
				coax_msgs::CoaxGetControlParameters::Response &out )
		{
			out.params.baseThrust = 0.5;
			out.params.yawOffset = 0.1;
			out.params.altitudeKp = 1;
			out.params.altitudeKi = 1e-3;
			out.params.altitudeKd = 1;
			out.params.yawKp = 2;
			out.params.yawKi = 2e-3;
			out.params.yawKd = 2e-1;
			ROS_INFO("Get control parameters [%d]", 0);
			return true;
		}

		bool set_control_parameters(coax_msgs::CoaxSetControlParameters::Request  &req,
				coax_msgs::CoaxSetControlParameters::Response &out )
		{
			out.result = -1;
			ROS_INFO("Set control parameters [%d]", res);
			return true;
		}

		bool get_trim_mode(coax_msgs::CoaxGetTrimMode::Request  &req,
				coax_msgs::CoaxGetTrimMode::Response &out )
		{
			out.mode.trimMode = 0;
			out.mode.rollTrim = 1;
			out.mode.pitchTrim = 2;
			ROS_INFO("Get trim mode [%d]", 0);
			return true;
		}

		bool set_trim_mode(coax_msgs::CoaxSetTrimMode::Request  &req,
				coax_msgs::CoaxSetTrimMode::Response &out )
		{
			out.result = -1;
			ROS_INFO("Set trim mode [%d]", out.result);
			return true;
		}

		bool get_sensor_list(coax_msgs::CoaxGetSensorList::Request  &req,
				coax_msgs::CoaxGetSensorList::Response &out )
		{
			out.list = 0xDEADBEEF;
			return true;
		}

		bool request_state(coax_msgs::CoaxRequestState::Request  &req,
				coax_msgs::CoaxRequestState::Response &out )
		{
            out.state = stateMsg;
			return true;
		}

		bool reach_nav_state(coax_msgs::CoaxReachNavState::Request  &req,
				coax_msgs::CoaxReachNavState::Response &out )
		{
			ROS_INFO("Reaching nav state %s [%f]",sbNavModeString(req.desiredState),req.timeout);
			out.result = -1;
			ROS_INFO("Reached nav state %s [%d]",sbNavModeString(req.desiredState),out.result);
			return true;
		}

		bool set_ack_mode(coax_msgs::CoaxSetAckMode::Request  &req,
				coax_msgs::CoaxSetAckMode::Response &out )
		{
			ROS_INFO("set ack mode [%d]", req.mode);
			out.result = -1;
			return true;
		}

		bool set_timeout(coax_msgs::CoaxSetTimeout::Request  &req,
				coax_msgs::CoaxSetTimeout::Response &out )
		{
			ROS_INFO("set timeout [%04X %04X]", req.control_timeout_ms,req.watchdog_timeout_ms);
			out.result = -1;
			return true;
		}

		bool send_string(coax_msgs::CoaxSendString::Request  &req,
				coax_msgs::CoaxSendString::Response &out )
		{
			ROS_INFO("sending string [%s]", req.text.c_str());
			out.result = -1;
			return true;
		}

		bool set_verbose(coax_msgs::CoaxSetVerbose::Request  &req,
				coax_msgs::CoaxSetVerbose::Response &out )
		{
			out.result = -1;
			ROS_INFO("Set verbosity [%d]", req.verbose);
			return true;
		}

		bool set_control(coax_msgs::CoaxSetControl::Request  &req,
				coax_msgs::CoaxSetControl::Response &out )
		{
			out.result = -1;
			ROS_INFO("Set Control [%d]", out.result);
			return true;
		}

		bool set_raw_control(coax_msgs::CoaxSetRawControl::Request  &req,
				coax_msgs::CoaxSetRawControl::Response &out )
		{
			out.result = -1;
			ROS_INFO("Set Control [%d]", out.result);
			return true;
		}

		bool reset(coax_msgs::CoaxReset::Request  &req,
				coax_msgs::CoaxReset::Response &out )
		{
			out.result = -1;
			ROS_INFO("Reset [%d]", out.result);
			return true;
		}

		void control_callback(const coax_msgs::CoaxControl::ConstPtr & message) {
            printf("C");fflush(stdout);
		}

		void raw_control_callback(const coax_msgs::CoaxRawControl::ConstPtr & message) {
            printf("!");fflush(stdout);
		}
};

int main(int argc, char **argv)
{
	int res;

	ros::init(argc, argv, "coax_server");
	SBController api;
	CRITICAL(res = api.initialise((argc<2)?("localhost"):(argv[1])));

	ros::NodeHandle n("/coax_server");
    ros::Timer timer = n.createTimer(ros::Duration(1. / 20.), 
            &SBController::timerCallback,&api);

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

