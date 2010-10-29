#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include "coax_msgs/CoaxConfigureComm.h"
#include "coax_msgs/CoaxConfigureControl.h"
#include "coax_msgs/CoaxReachNavState.h"
#include "coax_msgs/CoaxSetControl.h"
#include "coax_msgs/CoaxSetTimeout.h"

#include "coax_msgs/Coax3DSetControlMode.h"

#include "coax_msgs/CoaxConstants.h"

#include "coax_msgs/CoaxState.h"
#include "coax_msgs/CoaxControl.h"

class Coax3DController
{
	protected:
		typedef coax_msgs::Coax3DSetControlMode::Request _SCM;

		std::vector<ros::ServiceServer> services;
		
		ros::Publisher control_pub;

		ros::Subscriber pose_sub;
		ros::Subscriber state_sub;
		ros::Subscriber desired_twist_sub;
		ros::Subscriber desired_position_sub;

		ros::ServiceClient cfgControlClt;
		ros::ServiceClient cfgCommClt;
		ros::ServiceClient reachNavStateClt;

		bool activateControl;
		unsigned int controlMode;
		// Constant to save altitude in transitions
		float hoverAltitude;

		// Measured position and orientation
		geometry_msgs::PoseStamped latestPose;
		coax_msgs::CoaxState latestState;

		// Desired control if defined in term of linear and angular velocity
		geometry_msgs::TwistStamped desiredTwist;

		// Desired control if defined in term of position
		geometry_msgs::PointStamped desiredPosition;
	public:
		Coax3DController(ros::NodeHandle & n) 
		{
			services.push_back(n.advertiseService("coax_3d/set_control_mode", &Coax3DController::set_control_mode, this));

			// Publishing the control
			control_pub = n.advertise<coax_msgs::CoaxControl>("coax_server/control",10);

			// Subscribing to position message 
			ros::TransportHints hints;
			// Subscribe to these messages in UDP/unreliable mode: we need more
			// speed than reliability for them
			pose_sub = n.subscribe("coax_3d/pose",10,&Coax3DController::pose_callback,this,hints.udp());
			state_sub = n.subscribe("coax_server/state",10,&Coax3DController::state_callback,this,hints.udp());

			// Subscribe to the desired position/twist in TCP. We want to be
			// sure not to miss any set point.
			desired_twist_sub = n.subscribe("coax_3d/desired_twist",10,&Coax3DController::desired_twist_callback,this,hints.reliable());
			desired_position_sub = n.subscribe("coax_3d/desired_position",10,&Coax3DController::desired_position_callback,this,hints.reliable());

			cfgControlClt = n.serviceClient<coax_msgs::CoaxConfigureControl>("coax_server/configure_control");
			cfgCommClt = n.serviceClient<coax_msgs::CoaxConfigureComm>("coax_server/configure_comm");
			reachNavStateClt = n.serviceClient<coax_msgs::CoaxReachNavState>("coax_server/reach_nav_state");

			activateControl = false;

			// Initialise the control state to sensible values
			controlMode = 0;
			hoverAltitude = 0;

			desiredTwist.header.stamp = ros::Time::now();
			desiredTwist.twist.linear.x = 0;
			desiredTwist.twist.linear.y = 0;
			desiredTwist.twist.linear.z = 0;
			desiredTwist.twist.angular.x = 0;
			desiredTwist.twist.angular.y = 0;
			desiredTwist.twist.angular.z = 0;

			desiredPosition.header.stamp = ros::Time::now();
			desiredPosition.point.x = 0;
			desiredPosition.point.y = 0;
			desiredPosition.point.z = 0;
		}

		~Coax3DController() {
		}

		bool configure_control() {
			coax_msgs::CoaxConfigureControl srv;
			// TODO: when other modes will have been implemented
			// instantiate them based on controlMode
			srv.request.rollMode = coax_msgs::CoaxConstants::SB_CTRL_POS;
			srv.request.pitchMode = coax_msgs::CoaxConstants::SB_CTRL_POS;
			srv.request.yawMode = coax_msgs::CoaxConstants::SB_CTRL_VEL;
			srv.request.altitudeMode = coax_msgs::CoaxConstants::SB_CTRL_REL;
			if (cfgControlClt.call(srv))
			{
				ROS_INFO("Configure control: %ld", (long int)srv.response.result);
			}
			else
			{
				ROS_ERROR("Failed to call service configure control");
				return false;
			}
			return true;
		}

		bool configure_comm() {
			coax_msgs::CoaxConfigureComm srv;
			srv.request.commMode = coax_msgs::CoaxConstants::SB_COM_CONTINUOUS;
			srv.request.frequency = 50;
			srv.request.numMessages = 1000;
			srv.request.contents = coax_msgs::CoaxConstants::SBS_ALL;
			if (cfgCommClt.call(srv))
			{
				ROS_INFO("Configure comm: %ld", (long int)srv.response.result);
			}
			else
			{
				ROS_ERROR("Failed to call service configure comm");
				return false;
			}
			return true;
		}

		bool reach_nav_state(unsigned int state,float timeout=30.0) {
			coax_msgs::CoaxReachNavState srv;
			srv.request.desiredState = state;
			srv.request.timeout = timeout;
			if (reachNavStateClt.call(srv))
			{
				ROS_INFO("Reach nav state: %ld", (long int)srv.response.result);
			}
			else
			{
				ROS_ERROR("Failed to call service reach nav state");
				return false;
			}
			return true;
		}



		bool set_control_mode(coax_msgs::Coax3DSetControlMode::Request  &req,
				coax_msgs::Coax3DSetControlMode::Response &out )
		{
			out.result = 0;
			configure_comm();
			hoverAltitude = latestState.zrange;
			switch (req.mode) {
				case _SCM::velocityMode:
					controlMode = req.mode;
					configure_control();
					desiredTwist.header.stamp = ros::Time::now();
					desiredTwist.twist.linear.x = 0;
					desiredTwist.twist.linear.y = 0;
					desiredTwist.twist.linear.z = 0;
					desiredTwist.twist.angular.x = 0;
					desiredTwist.twist.angular.y = 0;
					desiredTwist.twist.angular.z = 0;
					break;
				case _SCM::positionMode:
					controlMode = req.mode;
					configure_control();
					desiredPosition.point = latestPose.pose.position;
					break;
				case _SCM::stopMode:
					controlMode = req.mode;
					reach_nav_state(coax_msgs::CoaxConstants::SB_NAV_STOP);
					break;
				case _SCM::nocontrolMode:
					controlMode = req.mode;
					configure_control();
					break;
				default:
					ROS_ERROR("Set control mode: invalid mode[%d]", out.result);
					out.result = -1;
					return false;
			}
			if (controlMode > _SCM::stopMode) {
				reach_nav_state(coax_msgs::CoaxConstants::SB_NAV_CTRLLED);
			}
			ROS_INFO("Set control mode [%d]", out.result);
			return true;
		}

		void pose_callback(const geometry_msgs::PoseStamped::ConstPtr & message) {
			latestPose = *message;
			// TODO: implement some kalman filter here?
		}

		void state_callback(const coax_msgs::CoaxState::ConstPtr & message) {
			latestState = *message;
			// TODO: implement some kalman filter here?
		}

		void desired_twist_callback(const geometry_msgs::TwistStamped::ConstPtr & message) {
			desiredTwist = *message;
		}

		void desired_position_callback(const geometry_msgs::PointStamped::ConstPtr & message) {
			desiredPosition = *message;
		}

		void run(unsigned int rate) {
			double K = 1.0;
			ros::Rate loop_rate(rate);
			while (ros::ok())
			{
				coax_msgs::CoaxControl control;
				switch (controlMode) {
					case _SCM::velocityMode:
						// TODO: compute roll and pitch based on desired
						// linear velocity
						control.roll = K * desiredTwist.twist.linear.y; 
						control.pitch = K * desiredTwist.twist.linear.x; 
						control.yaw = desiredTwist.twist.angular.z = 0;
						// TODO: compute desired altitude based on desired
						// linear velocity, or use the right mode 
						// in the low-level control
						control.altitude = hoverAltitude;
						control_pub.publish(control);
						break;
					case _SCM::positionMode:
						// TODO: compute roll and pitch based on error between
						// desired pose and current pose
						control.roll = K * (latestPose.pose.position.y - 
								desiredPosition.point.y);
						control.pitch = K * (latestPose.pose.position.x - 
								desiredPosition.point.x);
						// Yaw is set to zero in position mode for now.
						// Should we control it to face toward the target
						// position
						control.yaw = 0;
						control.altitude = desiredPosition.point.z;
						control_pub.publish(control);
						break;
					case _SCM::stopMode:
						control.roll = 0;
						control.pitch = 0;
						control.yaw = 0;
						control.altitude = 0;
						// We don't need to publish in this mode
						break;
					case _SCM::nocontrolMode:
						// idle mode: no control, but we keep publishing to
						// prevent watchdog timeout
						control.roll = 0;
						control.pitch = 0;
						control.yaw = 0;
						control.altitude = hoverAltitude;
						control_pub.publish(control);
						break;
					default:
						break;
				}
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "coax_3d_controller");
	ros::NodeHandle n;

	Coax3DController api(n);


	ROS_INFO("Coax 3D controller ready");

	api.run(50);

	return 0;
}

