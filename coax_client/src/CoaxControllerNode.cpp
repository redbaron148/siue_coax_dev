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
#include <coax_msgs/CoaxConfigureOAMode.h>
#include <coax_msgs/CoaxConfigureComm.h>
#include <coax_msgs/CoaxConfigureControl.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_msgs/CoaxSetTimeout.h>
#include <coax_msgs/CoaxControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>

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

#define DEFAULT_NAV_STATE_TIMEOUT   5.0
#define DEFAULT_AUTO_POSE_TIMEOUT   2.0
#define DEFAULT_MAX_AUTO_ROLL       0.065
#define DEFAULT_MAX_AUTO_PITCH      0.065
#define DEFAULT_PITCH_P_VALUE       0.06
#define DEFAULT_ROLL_P_VALUE        0.06

static int end = 0;

void sighdl(int n) {
    end ++;
}


class SBController
{
    protected:
        boost::shared_ptr<joy::Joy> joystate;
        boost::shared_ptr<coax_msgs::CoaxState> state;
        boost::shared_ptr<geometry_msgs::PoseStamped> current_pose;
        boost::shared_ptr<geometry_msgs::PoseStamped> previous_pose;
        boost::shared_ptr<geometry_msgs::Pose2D> current_goal;
        ros::Subscriber state_sub;
        ros::Subscriber joy_sub;
        ros::Subscriber heli_pose_sub;
        ros::Subscriber heli_goal_sub;
        ros::Publisher control_pub;
        ros::ServiceClient cfgControlClt;
        ros::ServiceClient cfgCommClt;
        ros::ServiceClient cfgOAClt;
        ros::ServiceClient reachNavStateClt;
        ros::ServiceClient setTimeoutClt;

        bool firstctrl,gotjoy,automode,gotpose;
        float delta_x,delta_y;
        double roll_p, pitch_p, max_pitch, max_roll;
        double auto_pose_timeout, nav_state_timeout;
    public:
        SBController() {
            firstctrl = true;
            gotjoy = false;
            automode = false;
            gotpose = false;
            current_goal = boost::shared_ptr<geometry_msgs::Pose2D>(new geometry_msgs::Pose2D());
            current_goal->x = .5;
            current_goal->y = .5;
        }
        ~SBController() {
        }

        void joyCallback(boost::shared_ptr<joy::Joy> msg) {
            //printf("Got Joy\n");
            assert(msg->buttons.size()>4);
            assert(msg->axes.size()>=2);
            joystate = msg;
            gotjoy = true;
        }
        
        void stateCallback(boost::shared_ptr<coax_msgs::CoaxState> msg) {
            // printf("Got State\n");
            state = msg;
        }

        void goalPoseCallback(boost::shared_ptr<geometry_msgs::Pose2D> msg){
            current_goal = msg;
        }
        
        void heliPoseCallback(boost::shared_ptr<geometry_msgs::PoseStamped> msg){
            //std::cout << "got pose" << std::endl;
            previous_pose = current_pose;
            current_pose = msg;
            gotpose = true;
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

        void joyctrl() {
            ros::Rate looprate(10);
            int res = 0;
            float desHeight=0.0;
            double desYaw = 0;
            double desRoll = 0;
            double desPitch = 0;
            unsigned int pose_count = 0;

            while (ros::ok()) {
                //ROS_INFO("trying a joyctrl");
                looprate.sleep();
                ros::spinOnce();
                desYaw = 0;
                
                if (!gotjoy || state == NULL) continue;
                //joy_count = 0;

                if (state->errorFlags) {
                    printf("An error has been detected on the PIC: %02X\n",
                            state->errorFlags);
                    DEBUG(res = reachNavState(SB_NAV_STOP,nav_state_timeout));
                    return;
                }
                
                //if autoflight is enabled
                if(automode){
                    if(current_goal != NULL && current_pose != NULL &&
                    (ros::Time::now()-current_pose->header.stamp).toSec()<=auto_pose_timeout){
                        if(gotpose){
                            delta_y = (-current_goal->y+current_pose->pose.position.y);
                            delta_x = (current_goal->x-current_pose->pose.position.x);
                            pose_count=0;
                            //gotpose = false;
                        }
                        else {
                            if(pose_count >= 5)
                                ROS_INFO("have not gotten a new pose recently, may timeout");
                            pose_count++;
                        }
                        ROS_WARN("pose: (%f,%f)",current_pose->pose.position.x,current_pose->pose.position.y);
                        ROS_WARN("goal: (%f,%f)\n",current_goal->x,current_goal->y);
                        ROS_WARN("delta_x: %f  delta_y: %f",delta_x,delta_y);
                        desPitch = delta_x*-pitch_p;
                        desRoll = delta_y*roll_p;
                        if(desRoll >= 0) desRoll = MIN(max_roll,desRoll);
                        else desRoll = MAX(-max_roll,desRoll);
                        if(desPitch >= 0) desPitch = MIN(max_pitch,desPitch);
                        else desPitch = MAX(-max_pitch,desPitch);
                        ROS_INFO("desPitch: %f  desRoll: %f",desPitch,desRoll);
                    }
                    else{
                        ROS_WARN("Cannot auto fly, transitioning to IDLE.");
                        if(current_goal == NULL) ROS_WARN("No Goal");
                        else if(current_pose == NULL) ROS_WARN("No Pose");
                        else ROS_WARN("timed out: %f - %f = %f",ros::Time::now().toSec(),current_pose->header.stamp.toSec(),(ros::Time::now()-current_pose->header.stamp).toSec());
                        if(automode) ROS_INFO("turning auto mode off");
                        DEBUG(res = reachNavState(SB_NAV_IDLE,nav_state_timeout));
                        ROS_INFO("Transition to IDLE completed");
                        automode = false;
                        break;
                    }
                }

                switch (state->mode.navigation) {
                    case SB_NAV_STOP:
                        desHeight = 0;
                        firstctrl = true;
                        if (joystate->buttons[0]) {
                            DEBUG(res = reachNavState(SB_NAV_IDLE,nav_state_timeout));
                            ROS_INFO("Transition to IDLE completed");
                        }
                        break;
                    case SB_NAV_IDLE:
                        desHeight = 0;
                        firstctrl = true;
                        if (joystate->buttons[0]) {
                            if (joystate->axes[2] > -0.8) {
                                ROS_INFO("Refusing transition to controlled while the height axis (%.2f) is above -0.8",joystate->axes[2]);
                            } else {
                                DEBUG(res = reachNavState(SB_NAV_CTRLLED,nav_state_timeout));
                                ROS_INFO("Transition to CTRLLED completed");
                            }
                        }
                        if (joystate->buttons[1]) {
                            DEBUG(res = reachNavState(SB_NAV_STOP,nav_state_timeout));
                            ROS_INFO("Transition to STOP completed");
                        }
                        break;
                    case SB_NAV_TAKEOFF:
                    case SB_NAV_LAND:
                    case SB_NAV_HOVER:
                    case SB_NAV_SINK:
                        firstctrl = true;
                        desHeight = state->zrange;
                        if (joystate->buttons[1]) {
                            DEBUG(res = reachNavState(SB_NAV_IDLE,nav_state_timeout));
                            ROS_INFO("Transition to IDLE completed");
                        }
                        break;
                    case SB_NAV_CTRLLED:
                    {
                        if (firstctrl) {
                            desHeight = state->zrange;
                            printf("Initial control: desHeight = %f\n",desHeight);
                            firstctrl = false;
                        }
                        if (joystate->buttons[0]) {
                            DEBUG(res = reachNavState(SB_NAV_IDLE,nav_state_timeout));
                            ROS_INFO("Transition to IDLE completed");
                            automode = false;
                            break;
                        }
                        if(joystate->buttons[1]){
                            automode = false;
                            ROS_INFO("turning auto mode off");
                        }
                        
                        if(joystate->buttons[2]){
                            automode = true;
                            ROS_INFO("turning auto mode on");
                        }
                        
                        desHeight = ((1 + joystate->axes[2])/2) * 1.0;
                        if(!automode)
                        {
                            if (joystate->buttons[3]) { desYaw = -80*M_PI/180; }
                            if (joystate->buttons[4]) { desYaw = +80*M_PI/180.; }
                            desPitch = -(0.25*joystate->axes[1]);
                            desRoll = -(0.25*joystate->axes[0]);
                        }
                        DEBUG(res = setControl(desRoll,desPitch,desYaw,desHeight));
                        ROS_INFO("Battery: %f GyroZ: %f Joy %.3f %.3f",
                                state->battery,
                                state->gyro[2], 
                                desHeight, state->zrange);
                        break;
                    }
                    default:
                        break;
                }
                
                if(gotpose)
                    gotpose = false;
            }
        }

        int initialise(ros::NodeHandle & n) {
            std::string coax_server = "/coax_server";
            std::string joytopic = "/joy";
            std::string goal_node_prefix = "";
            std::string pose_node_prefix = "/blob_mapper";
            // n.param<std::string>("coax_server",coax_server,coax_server);
            // n.param<std::string>("joy_topic",joytopic,joytopic);
            
            n.param("pitch_p_value", pitch_p, DEFAULT_PITCH_P_VALUE);
            n.param("roll_p_value", roll_p, DEFAULT_ROLL_P_VALUE);
            n.param("max_pitch", pitch_p, DEFAULT_MAX_AUTO_PITCH);
            n.param("max_roll", max_pitch, DEFAULT_MAX_AUTO_ROLL);
            n.param("nav_state_timeout", nav_state_timeout, DEFAULT_NAV_STATE_TIMEOUT);
            n.param("auto_pose_timeout", auto_pose_timeout, DEFAULT_AUTO_POSE_TIMEOUT);

            cfgControlClt = n.serviceClient<coax_msgs::CoaxConfigureControl>(coax_server+"/configure_control");
            cfgCommClt = n.serviceClient<coax_msgs::CoaxConfigureComm>(coax_server+"/configure_comm");
            cfgOAClt = n.serviceClient<coax_msgs::CoaxConfigureOAMode>(coax_server+"/configure_oamode");
            reachNavStateClt = n.serviceClient<coax_msgs::CoaxReachNavState>(coax_server+"/reach_nav_state");
            setTimeoutClt = n.serviceClient<coax_msgs::CoaxSetTimeout>(coax_server+"/set_timeout");

            // Subscribe to the state
            ros::TransportHints hints;
            state_sub = n.subscribe(coax_server+"/state",1,&SBController::stateCallback, this, hints.udp());
            heli_pose_sub = n.subscribe("/blob_mapper/helicopter_pose",1,&SBController::heliPoseCallback,this);
            heli_goal_sub = n.subscribe("/goal",1,&SBController::goalPoseCallback,this);
            // state_sub = n.subscribe(coax_server+"/state",1,&SBController::stateCallback, this);
            // Publishing the control
            control_pub = n.advertise<coax_msgs::CoaxControl>(coax_server+"/control",10);

            joy_sub = n.subscribe(joytopic,1,&SBController::joyCallback, this);

            ROS_INFO("Coax namespace %s joy topic %s",coax_server.c_str(),joytopic.c_str());
            ROS_INFO("Listening for heli poses: %s",(pose_node_prefix+"/helicopter_pose").c_str());
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
    
    //ROS_INFO("and not here");

    api.joyctrl();
    
    //ROS_INFO("and certainly not here");

    ros::shutdown();

    return 0;
}

        
