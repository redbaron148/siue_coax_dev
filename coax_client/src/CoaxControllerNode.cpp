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

/**
 *  @file   CoaxController.cpp
 *  @author Aaron Parker
 *  @date   03-24-2011
 *  @brief  Controller for the CoaX helicopter robot, running ROS, assumes 
 *          global pose data is available.
 */

#define switch_nav_mode_key "n"
#define stop_key            "s"
#define increase_pitch_key  "k"
#define decrease_pitch_key  "i"
#define increase_roll_key   "l"
#define decrease_roll_key   "j"
#define increase_height_key "u"
#define decrease_height_key "o"

#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#include <ros/ros.h>
//#include <joy/Joy.h>
// Just for constants, and their text conversion
#include <com/sbapi.h>

#include <coax_msgs/CoaxState.h>
#include <coax_msgs/CoaxConfigureOAMode.h>
#include <coax_msgs/CoaxConfigureComm.h>
#include <coax_msgs/CoaxConfigureControl.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_msgs/CoaxSetTimeout.h>
#include <coax_msgs/CoaxControl.h>
#include <coax_client/Keyboard.h>

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
        //joy::Joy joystate;
        coax_client::Keyboard key;
        boost::shared_ptr<coax_msgs::CoaxState> state;
        ros::Subscriber state_sub;
        ros::Subscriber key_sub;
        ros::Publisher control_pub;
        ros::ServiceClient cfgControlClt;
        ros::ServiceClient cfgCommClt;
        ros::ServiceClient cfgOAClt;
        ros::ServiceClient reachNavStateClt;
        ros::ServiceClient setTimeoutClt;

        bool firstctrl,gotkey;
    public:
        SBController() {
            firstctrl = true;
            gotkey = false;
            //joystate.buttons.resize(12);
            //joystate.axes.resize(12);
        }
        ~SBController() {
        }

        void stateCallback(boost::shared_ptr<coax_msgs::CoaxState> msg) {
            // printf("Got State\n");
            state = msg;
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

        int setControl(float roll, float pitch, float yaw, float altitude) {
            coax_msgs::CoaxControl control;
            control.roll = roll;
            control.pitch = pitch;
            control.yaw = yaw;
            control.altitude = altitude;
            control_pub.publish(control);
            return 0;
        }

        void keyboardCallback(const coax_client::Keyboard::ConstPtr& msg) {
            //assert(msg->buttons.size()>4);
            //assert(msg->axes.size()>=2);
            //joystate = *msg;
            gotkey = true;
            key = *msg;
        }

        void autoctrl() {
            ros::Rate looprate(10);
            int res = 0;
            float desHeight=0.0;

            while (ros::ok()) {
                looprate.sleep();
                ros::spinOnce();
                if (!gotkey) continue;

                if (state->errorFlags) {
                    printf("An error has been detected on the PIC: %02X\n",
                            state->errorFlags);
                    DEBUG(res = reachNavState(SB_NAV_STOP,30.0));
                    return;
                }

                switch (state->mode.navigation) 
                {
                    case SB_NAV_STOP:
                        desHeight = 0;
                        firstctrl = true;
                        if (key.key==switch_nav_mode_key) {
                            DEBUG(res = reachNavState(SB_NAV_IDLE,30.0));
                            ROS_INFO("Transition to IDLE completed");
                        }
                        break;
                    case SB_NAV_IDLE:
                        desHeight = 0;
                        firstctrl = true;
                        if (key.key==switch_nav_mode_key) {
                                DEBUG(res = reachNavState(SB_NAV_CTRLLED,30.0));
                                ROS_INFO("Transition to CTRLLED completed");
                            }
                        if (key.key==stop_key) {
                            DEBUG(res = reachNavState(SB_NAV_STOP,30.0));
                            ROS_INFO("Transition to STOP completed");
                        }
                        break;
                    case SB_NAV_TAKEOFF:
                    case SB_NAV_LAND:
                    case SB_NAV_HOVER:
                    case SB_NAV_SINK:
                        firstctrl = true;
                        desHeight = state->zrange;
                        if (key.key==stop_key) {
                            DEBUG(res = reachNavState(SB_NAV_IDLE,30.0));
                            ROS_INFO("Transition to IDLE completed");
                        }
                        break;
                    case SB_NAV_CTRLLED:
                        {
                            double desYaw = 0;
                            double desRoll = state->roll;
                            double desPitch = state->pitch;
                            //std::cout << "is controlled! " << key.key << std::endl;
                            if (firstctrl) {
                                desHeight = state->zrange;
                                printf("Initial control: desHeight = %f\n",desHeight);
                                firstctrl = false;
                            }
                            if (key.key==switch_nav_mode_key) {
                                DEBUG(res = reachNavState(SB_NAV_IDLE,30.0));
                                ROS_INFO("Transition to IDLE completed");
                                break;
                            }
                            if(key.key==stop_key)
                            {
                                DEBUG(res = reachNavState(SB_NAV_STOP,30.0));
                                ROS_INFO("Transition to IDLE completed");
                                break;
                            }
                            if(key.key=="q")
                            {
                                desRoll = 0;
                                desPitch = 0;
                            }
#if 1
                            if (key.key==decrease_height_key) { 
                                desHeight -= 2e-2; 
                                key.key = "q";
                                printf("Height: %f\n",desHeight);
                            }
                            else if (key.key==increase_height_key) {
                                desHeight += 2e-2; 
                                key.key = "q";
                                printf("Height: %f\n",desHeight);
                            }
                            else if (key.key==increase_roll_key)
                            {
                                desRoll += .01; 
                                key.key = "q";
                                printf("Roll: %f\n",desRoll);
                            }
                            else if (key.key==decrease_roll_key)
                            {
                                desRoll = -.01; 
                                key.key = "q";
                                printf("Roll: %f\n",desRoll);
                            }
                            else if (key.key==increase_pitch_key)
                            {
                                desPitch = .01; 
                                key.key = "q";
                                printf("Pitch: %f\n",desPitch);
                            }
                            else if (key.key==decrease_pitch_key)
                            {
                                desPitch = -.01; 
                                key.key = "q";
                                printf("Pitch: %f\n",desPitch);
                            }
#else
                            //desHeight = ((1 + joystate.axes[2])/2) * 1.0;
#endif
                            //if (joystate.buttons[3]) { desYaw = -80*M_PI/180; }
                            //if (joystate.buttons[4]) { desYaw = +80*M_PI/180.; }
                            // desYaw /= 17.; // IMU BUG
                            DEBUG(res = setControl(desRoll,desPitch,desYaw,desHeight));
                            /*ROS_INFO("Battery: %f GyroZ: %f Joy %.3f %.3f",
                                    state->battery,
                                    state->gyro[2], 
                                    desHeight, state->zrange);*/
                            break;
                        }
                    default:
                        break;
                }
                key.key = "q";
            }
        }

        int initialise(ros::NodeHandle & n) {
            std::string coax_server = "coax_server";
            std::string key_topic = "/keyboard";

            cfgControlClt = n.serviceClient<coax_msgs::CoaxConfigureControl>(coax_server+"/configure_control");
            cfgCommClt = n.serviceClient<coax_msgs::CoaxConfigureComm>(coax_server+"/configure_comm");
            cfgOAClt = n.serviceClient<coax_msgs::CoaxConfigureOAMode>(coax_server+"/configure_oamode");
            reachNavStateClt = n.serviceClient<coax_msgs::CoaxReachNavState>(coax_server+"/reach_nav_state");
            setTimeoutClt = n.serviceClient<coax_msgs::CoaxSetTimeout>(coax_server+"/set_timeout");

            // Subscribe to the state
            ros::TransportHints hints;
            state_sub = n.subscribe(coax_server+"/state",1,&SBController::stateCallback, this, hints.udp());
            control_pub = n.advertise<coax_msgs::CoaxControl>(coax_server+"/control",10);

            key_sub = n.subscribe(key_topic,1,&SBController::keyboardCallback, this);

            ROS_INFO("Coax namespace %s keyboard topic %s",coax_server.c_str(),key_topic.c_str());
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

    api.autoctrl();

    ros::shutdown();

    return 0;
}
