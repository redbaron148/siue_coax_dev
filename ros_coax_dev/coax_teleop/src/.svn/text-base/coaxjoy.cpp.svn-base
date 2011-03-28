/************************************************************
* 
* API and Communication library for the Skybotix AG
* helicopters, and particularly COAX
*
* Developed by Cedric Pradalier: cedric.pradalier@skybotix.ch
* Send modification or corrections as patches (diff -Naur)
* Copyright: Skybotix AG, 2009-2012
* 
* All rights reserved.
* 
* Skybotix API is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* Skybotix API is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with Skybotix API. If not, see <http://www.gnu.org/licenses/>.
* 
************************************************************/
#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#include <ros/ros.h>
#include <joy/Joy.h>
// Just for constants, and their text conversion
#include <com/sbapi.h>

#include <coax_msgs/CoaxState.h>
#include <coax_msgs/CoaxSetLight.h>
#include <coax_msgs/CoaxConfigureControl.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_msgs/CoaxSetTimeout.h>
#include <coax_msgs/CoaxControl.h>

#include <string>
#include <vector>

#define D2R(X) ((X)*M_PI/180.0)
#define R2D(X) ((X)*180.0/M_PI)
#define MAX(X,Y) (((X)<(Y))?(Y):(X))
#define MIN(X,Y) (((X)<(Y))?(X):(Y))

//#define DEBUG(c) printf("Executing "#c"\n")
//#define DEBUG(c) res=0;printf("Executing "#c"\n");c;printf("Result %d\n",res)
#define DEBUG(c) res=0;c;if (res) printf("Result of "#c": %d\n",res)
#define CRITICAL(c) res=0;c;if (res) {printf("Result of "#c": %d\n",res); return res;}

static int end = 0;

void sighdl(int n) {
	end ++;
}


class SBController
{
	protected:
		joy::Joy joystate;
		coax_msgs::CoaxState state;
		ros::Subscriber state_sub;
		ros::Subscriber joy_sub;
		ros::Publisher control_pub;
		ros::ServiceClient cfgControlClt;
		ros::ServiceClient setLightClt;
		ros::ServiceClient reachNavStateClt;
		ros::ServiceClient setTimeoutClt;

		bool firstctrl,gotjoy;
	public:
		SBController() {
			firstctrl = true;
            gotjoy = false;
            joystate.buttons.resize(12);
            joystate.axes.resize(12);
		}
		~SBController() {
		}

		void stateCallback(const coax_msgs::CoaxState::ConstPtr& msg) {
			// printf("Got State\n");
			state = *msg;
		}

		int reachNavState(unsigned int state, float timeout) {
			coax_msgs::CoaxReachNavState srv;
			srv.request.desiredState = state;
			srv.request.timeout = timeout;
			if (reachNavStateClt.call(srv))
			{
				ROS_INFO("Reach nav state: %ld", (long int)srv.response.result);
				return 0;
			}
			else
			{
				ROS_ERROR("Failed to call service reach nav state");
			}
			return -1;
		}

		int configureControl(unsigned roll, unsigned pitch, unsigned yaw, unsigned altitude) {
            coax_msgs::CoaxConfigureControl srv;
            srv.request.rollMode = roll;
            srv.request.pitchMode = pitch;
            srv.request.yawMode = yaw;
            srv.request.altitudeMode = altitude;
            if (cfgControlClt.call(srv))
            {
                ROS_INFO("Configure control: %ld", (long int)srv.response.result);
                return 0;
            }
            else
            {
                ROS_ERROR("Failed to call service configure control");
            }
            return -1;
        }

		int setLight(unsigned char percent) {
            coax_msgs::CoaxSetLight srv;
            srv.request.percent = percent;
            if (setLightClt.call(srv))
            {
                ROS_INFO("Set light: %ld", (long int)srv.response.result);
                return 0;
            }
            else
            {
                ROS_ERROR("Failed to call service set light");
            }
            return -1;
        }

		int setControl(float roll, float pitch, float yaw, float altitude) {
			coax_msgs::CoaxControl control;
			control.roll = roll;
			control.pitch = pitch;
			control.yaw = yaw;
			control.altitude = altitude;
			control_pub.publish(control);
			return 0;
		}

		void joyCallback(const joy::Joy::ConstPtr& msg) {
			// printf("Got Joy\n");
			assert(msg->buttons.size()>4);
			assert(msg->axes.size()>=2);
			joystate = *msg;
            gotjoy = true;
		}

		void joyctrl() {
            bool velmode = false;
			ros::Rate looprate(10);
			int res = 0;
			float desHeight=0.0;

			while (ros::ok()) {
				looprate.sleep();
				ros::spinOnce();
                if (!gotjoy) continue;
                if (state.errorFlags & SB_FLAG_MANUAL) {
                    // If we're not in auto, we can't do anything
                    if (state.mode.navigation != SB_NAV_STOP) {
                        // If we were not in stop, then go to stop, to 
                        // make sure we know in which state we are.
                        DEBUG(res = reachNavState(SB_NAV_STOP,30.0));
                    }
                    continue;
                }

				if (joystate.buttons[5] && 
                        (state.coaxspeed.state & COAXSPEED_AVAILABLE)) {
                    configureControl(SB_CTRL_VEL,SB_CTRL_VEL,
                            SB_CTRL_VEL, SB_CTRL_REL);
                    velmode = true;
                    ROS_INFO("Activated velocity control");
				}
				if (joystate.buttons[6] && 
                        (state.coaxspeed.state & COAXSPEED_AVAILABLE)) {
                    configureControl(SB_CTRL_POS,SB_CTRL_POS,
                            SB_CTRL_VEL, SB_CTRL_REL);
                    setLight(0);
                    velmode = false;
                    ROS_INFO("Disabled velocity control");
				}

#if 1
				if (state.errorFlags & (SB_FLAG_IMUCRASH | SB_FLAG_RCLOST)) {
					printf("An error has been detected on the PIC: %02X\n",
							state.errorFlags);
					DEBUG(res = reachNavState(SB_NAV_STOP,30.0));
					return;
				}
#endif

				switch (state.mode.navigation) {
					case SB_NAV_STOP:
						desHeight = 0;
						firstctrl = true;
						if (joystate.buttons[0]) {
							DEBUG(res = reachNavState(SB_NAV_CTRLLED,30.0));
                            ROS_INFO("Transition to CTRLLED completed");
                            if (state.coaxspeed.state & COAXSPEED_AVAILABLE) {
                                configureControl(SB_CTRL_VEL,SB_CTRL_VEL,
                                        SB_CTRL_VEL, SB_CTRL_REL);
                                velmode = true;
                                ROS_INFO("Activated velocity control");
                            }
						}
						break;
					case SB_NAV_IDLE:
					case SB_NAV_HOVER:
						desHeight = 0;
						firstctrl = true;
                        DEBUG(res = reachNavState(SB_NAV_CTRLLED,30.0));
                        ROS_INFO("Transition to CTRLLED completed");
						if (joystate.buttons[0]) {
							DEBUG(res = reachNavState(SB_NAV_STOP,30.0));
                            ROS_INFO("Transition to STOP completed");
						}
						break;
					case SB_NAV_TAKEOFF:
					case SB_NAV_LAND:
					case SB_NAV_SINK:
						firstctrl = true;
						desHeight = state.zrange;
						if (joystate.buttons[1]) {
							DEBUG(res = reachNavState(SB_NAV_STOP,30.0));
                            ROS_INFO("Transition to IDLE completed");
						}
						break;
					case SB_NAV_CTRLLED:
						{
							double desYaw = 0;
							double desRoll = 0;
							double desPitch = 0;
							if (firstctrl) {
								desHeight = state.zrange;
								printf("Initial control: desHeight = %f\n",desHeight);
								firstctrl = false;
							}
							if (joystate.buttons[0]) {
								DEBUG(res = reachNavState(SB_NAV_STOP,30.0));
                                ROS_INFO("Transition to STOP completed");
                                setLight(0);
								break;
							}
#if 0
							if (joystate.buttons[1]) { 
								desHeight -= 2e-2; 
								joystate.buttons[1] = 0;
								printf("Height: %f\n",desHeight);
							}
							if (joystate.buttons[2]) {
								desHeight += 2e-2; 
								joystate.buttons[2] = 0;
								printf("Height: %f\n",desHeight);
							}
#else
                            desHeight = ((1 + joystate.axes[2])/2) * 1.0;
#endif
							if (joystate.buttons[3]) { desYaw = -80*M_PI/180; }
							if (joystate.buttons[4]) { desYaw = +80*M_PI/180.; }
							// desYaw /= 17.; // IMU BUG
                            if (velmode) {
                                desPitch = -(0.50*joystate.axes[1]);
                                desRoll = -(0.50*joystate.axes[0]);
                            } else {
                                desPitch = -(0.25*joystate.axes[1]);
                                desRoll = -(0.25*joystate.axes[0]);
                            }
							DEBUG(res = setControl(desRoll,desPitch,desYaw,desHeight));
							ROS_INFO("Battery: %f GyroZ: %f Joy %.3f %.3f",
									state.battery,
									state.gyro[2], 
									desHeight, state.zrange);
							break;
						}
					default:
						break;
				}
			}
		}

		int initialise(ros::NodeHandle & n) {
            std::string coax_server = "coax_server";
            std::string joytopic = "/joy";
            n.param<std::string>("coax_server",coax_server,coax_server);
            n.param<std::string>("joy_topic",joytopic,joytopic);

			cfgControlClt = n.serviceClient<coax_msgs::CoaxConfigureControl>(coax_server+"/configure_control");
			setLightClt = n.serviceClient<coax_msgs::CoaxSetLight>(coax_server+"/set_light");
			reachNavStateClt = n.serviceClient<coax_msgs::CoaxReachNavState>(coax_server+"/reach_nav_state");
			setTimeoutClt = n.serviceClient<coax_msgs::CoaxSetTimeout>(coax_server+"/set_timeout");

			// Subscribe to the state
			ros::TransportHints hints;
			state_sub = n.subscribe(coax_server+"/state",1,&SBController::stateCallback, this, hints.udp());
			// state_sub = n.subscribe(coax_server+"/state",1,&SBController::stateCallback, this);
			// Publishing the control
			control_pub = n.advertise<coax_msgs::CoaxControl>(coax_server+"/control",10);

			joy_sub = n.subscribe(joytopic,1,&SBController::joyCallback, this);

            ROS_INFO("Coax namespace %s joy topic %s",coax_server.c_str(),joytopic.c_str());
            ROS_INFO("Coax Teleop initialised and ready to roll!");
			return 0;
		}


};

// #define SIMULATION

int main(int argc, char *argv[])
{
	int res = 0;
	ros::init(argc, argv, "coax_teleop");
	ros::NodeHandle n;

	SBController api;

	CRITICAL(res = api.initialise(n));

	api.joyctrl();

    ros::shutdown();

	return 0;
}

		
