/*************************************************************
 *
 * Low-level sensor and control library for the Skybotix AG
 * helicopters, and particularly COAX
 *
 * Developed by:
 *  * Samir Bouabdallah samir.bouabdallah@skybotix.ch
 *  * Cedric Pradalier: cedric.pradalier@skybotix.ch
 *  * Gilles Caprari: gilles.capraru@skybotix.ch
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
 * 
 * Description:This software is a control algorithm for the CoaX helicopter
 *************************************************************/
#include <math.h>
#include "configs/coax_ports.h"
#include "utils/utils.h"
#include "motor/motor.h"
#include "control/control.h"
#include "imu/imu.h"
#include "us/altitude.h"
#include "rc/coax_rc.h"
#include "rc/rc_statemachine.h"
#include "configs/coax_config.h"
#include "control/control-data.h"
#include "mouse/mouse.h"
#include "sharp/sharp.h"

//#define MANUAL_THROTTLE;		// Define if flight with throttle in manual and the yaw automatically controlled
#define USE_IMU_FFP			// Define if the fixed-point version of the IMU code is used
#define MOUSE_SENSOR
//#define USE_SHARP_ALTITUDE
//#define USE_DECOUPLED_CONTROL

#ifdef USE_IMU_FFP
#include "imu/imu_ffp.h"
#endif

#define YAW_SCALING 			3.0 // 0.8
#define ROLL_SCALING 			1.0
#define PITCH_SCALING			1.0
#define T_CTRL 					10		// [ms]
#define VOLTAGE_COMPENSATION

CONTROL_STATUS control;

int motor_off = 1;

static void resetPIDs()
{
    control.yaw_pid.e_old = 0.0;
    control.yaw_pid.u = 0.0;
    control.yaw_pid.i = 0.0;
    control.altitude_pid.e_old = 0.0;
    control.altitude_pid.u = 0.0;
    control.altitude_pid.i = 0.0;
    control.velocity_x_pid.e_old = 0.0;
    control.velocity_x_pid.u = 0.0;
    control.velocity_x_pid.i = 0.0;
    control.velocity_y_pid.e_old = 0.0;
    control.velocity_y_pid.u = 0.0;
    control.velocity_y_pid.i = 0.0;
}

void initControlVariables(void (*init_specific)(CONTROL_STATUS *))
{
    // Init Control Variables. ---------------------------------------------
    control.flags.last_CONTROL_MODE = 
        control.flags.CONTROL_MODE = CONTROL_STOP|CONTROL_MANUAL;
    control.flags.ABS_ALT_CTRL = 0;
    control.flags.THROTTLE = 0;

    control.motor.up = 0.0;
    control.motor.dw = 0.0;
    control.servo.servo1 = 0.0;
    control.servo.servo2 = 0.0;

    control.errors.IMU_REBOOTED = 0;
    control.errors.LOW_POWER_DETECTED = 0;

    control.flags.time_in_mode = -1;
    control.flags.time_in_idle = -1;

    control.roll.enable_auto = 0;
    control.roll.manual = 1;
    control.roll.velocity = 0;
    control.roll.ref = 0.0;	

    control.pitch.enable_auto = 0;
    control.pitch.manual = 1;
    control.pitch.velocity = 0;
    control.pitch.ref = 0.0;	

    control.yaw.enable_auto = 0;
    control.yaw.manual = 1;
    control.yaw.ref = 0.0;	

    control.altitude.enable_auto = 0;
    control.altitude.manual = 1;
    control.altitude.ref = 0.0;	

    // Control gains
    control.params.T_ctrl_s = (T_CTRL/1000.);
    control.params.T_ctrl_ms = (T_CTRL);
    control.params.Kh = 0.0;
    control.params.yawOffset = 0.0;	// %of top motor speed to be reduced (to reduce effort on yaw control)
    control.params.alt_min = 0;	//[m]
    control.params.alt_max = 1.5;	// max altitude authorized [m]
    control.params.alt_max_sensor = 4.0;	// max altitude supposed seen by the sensor [m]
    control.params.Kidle = 0.35;
    control.params.min_time_in_idle = (2000 / T_CTRL);
    control.params.max_time_in_inter = (1000 / T_CTRL);
    control.params.max_time_in_mode = (10000 / T_CTRL);

    control.yaw_pid.kp = 0.0;							
    control.yaw_pid.ki = 0.0;
    control.yaw_pid.kd = 0.0;

    control.altitude_pid.kp = 0.0;
    control.altitude_pid.ki = 0.0;
    control.altitude_pid.kd = 0.0;

    control.velocity_x_pid.kp = 0.0;
    control.velocity_x_pid.ki = 0.0;
    control.velocity_x_pid.kd = 0.0;

    control.velocity_y_pid.kp = 0.0;
    control.velocity_y_pid.ki = 0.0;
    control.velocity_y_pid.kd = 0.0;

    control.user_function = 0;
    control.user_context = 0;

    resetPIDs();

    // Now set the system specific parameters
    // from specific file in control-data/*.c
    if (init_specific) {
        init_specific(&control);
    } else {
        initSpecificControlParameters();
    }

    RCInitStateMachine();
    updateBatteryVoltage();
}


static void updateEnableFlags()
{
    BatteryVoltageEnum batteryState;

    // First increment counters
    if (control.flags.CONTROL_MODE == control.flags.last_CONTROL_MODE) {
        control.flags.time_in_mode ++;
        if (control.flags.time_in_mode > control.params.max_time_in_mode) {
            control.flags.time_in_mode = control.params.max_time_in_mode;
        }
    } else {
        control.flags.time_in_mode = 0;
        control.flags.last_CONTROL_MODE = control.flags.CONTROL_MODE;
    }

    if ((control.flags.CONTROL_MODE & 0x0F) == CONTROL_IDLE) {
        control.flags.time_in_idle ++;
        if (control.flags.time_in_idle > control.params.max_time_in_mode) {
            control.flags.time_in_idle = control.params.max_time_in_mode;
        }
    } else {
        control.flags.time_in_idle = 0;
    }


#if 1
    // Battery proctection feature, this assumes that the battery has been read
    // in the main program
    // updateBatteryVoltage(); this block the control loop
    batteryState = evalBatteryVoltageLEDAlert();
    if (control.errors.LOW_POWER_DETECTED && (batteryState == BATTERY_VOLTAGE_OK)) {
        // histeresis: it is not possible to fly again with low voltage without
        // a reset
        batteryState = BATTERY_VOLTAGE_LOW;
    }
    if ((control.flags.CONTROL_MODE & 0x0F) != CONTROL_IDLE) {
        switch (batteryState) {
            case BATTERY_VOLTAGE_CRITICAL:
                // This means we have less than 10.6 V. We switch off the motor
                // whatever the situation. We should have had enough time in
                // BATTERY_VOLTAGE_LOW to land
                LED_RED = 1;
                control.errors.LOW_POWER_DETECTED = 1;
                // This is connected out, because the voltage reading seems to be
                // too noisy...
                // control.flags.CONTROL_MODE = CONTROL_AUTO | CONTROL_STOP;
                // control.altitude.enable_auto = 0;
                // control.yaw.enable_auto = 0;
                // control.pitch.enable_auto = 0;
                // control.roll.enable_auto = 0;
                // control.altitude.ref = 0;
                // resetPIDs();
                return;
            case BATTERY_VOLTAGE_LOW:
                // This means we have less than 11V, but more than 10.6V. We land
                // slowly
                LED_RED = 1;
                // control.errors.LOW_POWER_DETECTED = 1;
                // This is connected out, because the voltage reading seems to be
                // too noisy...
                // CONTROL_MODE = (CONTROL_MODE & 0x0F) | CONTROL_AUTO;
                // alt.enable_auto = 1;
                // alt.ref = imuData.altitude - 1e-2;
                // return;
                break;
            case BATTERY_VOLTAGE_OK:
            default:
                break;
        }
    }
#endif

    switch (RCSMUpdateState()) {
        case RC_STOP:
            control.flags.CONTROL_MODE = CONTROL_MANUAL | CONTROL_STOP;
            break;
        case RC_AUTO:
            control.flags.CONTROL_MODE = CONTROL_AUTO 
                | (control.flags.CONTROL_MODE & 0x0F);
            break;
        case RC_MANUAL_IDLE:
            control.flags.CONTROL_MODE = CONTROL_MANUAL | CONTROL_IDLE;
            break;
        case RC_MANUAL_FLY_READY:
        case RC_MANUAL_FLY_RANGE:
        case RC_MANUAL_FLY_THRUST:
            control.flags.CONTROL_MODE = CONTROL_MANUAL | CONTROL_FLY;
            break;
        case RC_MANUAL_FLY_COUPLED:
            control.flags.CONTROL_MODE = CONTROL_MANUAL | CONTROL_FLY;
            break;
        case RC_MANUAL_FLY_COAXSPEED:
            control.flags.CONTROL_MODE = CONTROL_MANUAL | CONTROL_FLY;
            break;
        default:
        case RC_INITIALIZING:
        case RC_KILLED:
            break;
    }


    // Now that we've updated the global state, switch on the various parts 
    // of the controller.
    switch (control.flags.CONTROL_MODE & 0xF0) {
        case CONTROL_AUTO:
            // We are in autonomous mode. 
            control.roll.enable_auto = 1;
            control.pitch.enable_auto = 1;
            control.yaw.enable_auto = 1;	
            control.altitude.enable_auto = 1;
            break;
        case CONTROL_MANUAL:
            // We are in manual mode. 
            control.roll.enable_auto = 0;	
            control.pitch.enable_auto = 0;
            control.yaw.enable_auto = 0;
            control.altitude.enable_auto = 0;
            // Add here any setup for specific control mode
            switch (RCSMGetState()) {
                case RC_MANUAL_FLY_READY:
                    // Desactivate the control while we have not explicitly
                    // selected the control mode
                    control.roll.manual = 0;	
                    control.pitch.manual = 0;
                    control.roll.enable_auto = 1;	
                    control.pitch.enable_auto = 1;
                    control.roll.ref = 0;
                    control.pitch.ref = 0;
                    break;
                case RC_MANUAL_FLY_THRUST:
                    control.pitch.velocity = 0;
                    control.roll.velocity = 0;
                    control.flags.THROTTLE=1;
                    control.flags.COUPLED=0;
                    break;
                case RC_MANUAL_FLY_RANGE:
                    control.pitch.velocity = 0;
                    control.roll.velocity = 0;
                    control.flags.THROTTLE=0;
                    control.flags.COUPLED=0;
                    break;
                case RC_MANUAL_FLY_COUPLED:
                    control.pitch.velocity = 0;
                    control.roll.velocity = 0;
                    control.flags.THROTTLE=0;
                    control.flags.COUPLED=1;
                    break;
                case RC_MANUAL_FLY_COAXSPEED:
                    control.pitch.velocity = 1;
                    control.roll.velocity = 1;
                    control.flags.THROTTLE=0;
                    control.flags.COUPLED=1;
                    break;
                default:
                    control.pitch.velocity = 0;
                    control.roll.velocity = 0;
                    control.flags.THROTTLE=0;
                    control.flags.COUPLED=0;
                    break;
            }
            break;
        default:
            break;
    }

}

int CONTROL_request_transition(unsigned int newstate)
{
    newstate = newstate & 0x0F;
    // Requesting transition is only allowed in automatic mode
    if ((control.flags.CONTROL_MODE & 0xF0) != CONTROL_AUTO) {
        return -1;
    }
    // If we're already in the requested mode, ignore request
    if ((control.flags.CONTROL_MODE & 0x0F) == newstate) {
        return 0;
    }
    // Particular case, if low power has been detected
    if (control.errors.LOW_POWER_DETECTED) {
        switch (newstate) {
            case CONTROL_FLY:
            case CONTROL_RAW:
                // If we've detected low power, ignore flying request
                return 0;
            case CONTROL_IDLE:
                // If we've detected low power, ignore idle request from stop
                if ((control.flags.CONTROL_MODE & 0x0F) == CONTROL_STOP) {
                    return 0;
                }
                break;
            default:
                break;
        }
    }

    switch (control.flags.CONTROL_MODE & 0x0F) {
        case CONTROL_RAW:
            // From raw, we can only STOP. Managing other transitions
            // is just too unsafe
            switch (newstate) {
                case CONTROL_STOP:
                    control.flags.CONTROL_MODE = CONTROL_AUTO | CONTROL_STOP;
                default:
                    return -1;
            }
            break;
        case CONTROL_STOP:
            // From stop we can only go to idle
            switch (newstate) {
                case CONTROL_IDLE:
                    control.flags.CONTROL_MODE = CONTROL_AUTO | CONTROL_IDLE;
                    break;
                case CONTROL_RAW:
                    // Raw transition is only allowed from STOP
                    control.flags.CONTROL_MODE = CONTROL_AUTO | CONTROL_RAW;
                    break;
                default:
                    return -1;
            }
            break;
        case CONTROL_IDLE:
            switch (newstate) {
                case CONTROL_STOP:
                    // From idle to stop is allowed directly
                    control.flags.CONTROL_MODE = CONTROL_AUTO | CONTROL_STOP;
                    break;
                case CONTROL_FLY:
                    // From idle to fly must wait min_time_in_idle
                    // in order to let the rotors get up to speed
                    if (control.flags.time_in_idle > control.params.min_time_in_idle) {
                        control.flags.CONTROL_MODE = CONTROL_AUTO | CONTROL_FLY;
                    } else {
                        return +1;
                    }
                    break;
                default:
                    return -1;
            }
            break;
        case CONTROL_FLY:
            // From fly to anything we need to go through idle
            switch (newstate) {
                case CONTROL_STOP:
                case CONTROL_IDLE:
                    control.flags.CONTROL_MODE = CONTROL_AUTO | CONTROL_IDLE;
                default:
                    return -1;
            }
            break;
        default:
            return -1;
    }
    return 0;
}

// This should be enough...
//#define absolute(x) (((x)<0)?(-(x)):(x))

void control_set_servo_decoupled(float roll, float pitch) 
{
#ifdef USE_DECOUPLED_CONTROL
    if (control.flags.COUPLED) {
        float su2 = ((mouse.gyrofilt_delta_x>=0)?+1:-1) * mouse.gyrofilt_delta_x * mouse.gyrofilt_delta_x;
        float sv2 = ((mouse.gyrofilt_delta_y>=0)?+1:-1) * mouse.gyrofilt_delta_y * mouse.gyrofilt_delta_y;
        float u_serv1_decoup = 1.0507*(roll+pitch*(-0.3070)+0.07896678*su2);
        float u_serv2_decoup = 1.0507*(pitch+roll*(-0.3070)-0.07896678*sv2);
        MOTOR_SetServoAngle(u_serv1_decoup,u_serv2_decoup);
    } else {
        MOTOR_SetServoAngle(roll,pitch);
    }
#else
    MOTOR_SetServoAngle(roll,pitch);
#endif
}


void auto_control(void)
{
    float altitude_CTRL = 0.;
    float gyro_CTRL = 0.;
    float yaw_u_OP = 0.;


    // Misc. Parameters ----------------------------------------------------

#if 0
    altitude_CTRL = imuData.altitude; 		
#else
    if (control.flags.ABS_ALT_CTRL != control.flags.last_ABS_ALT_CTRL) {
        control.altitude_pid.e_old = 0.0;
        control.altitude_pid.u = 0.0;
        control.altitude_pid.i = 0.0;
        control.flags.last_ABS_ALT_CTRL = control.flags.ABS_ALT_CTRL;
    }
    if (control.flags.ABS_ALT_CTRL) {
        altitude_CTRL = imuData.abs_altitude; 		
    } else {
#ifdef USE_SHARP_ALTITUDE
        updateSHARP_LEFT();
        //filter the sharp signal 1.59=1/(2*pi*f_c) f_c:0.1
        altitude_CTRL = CONTROL_filter(sharp.left, altitude_CTRL,control.params.T_ctrl_s,0.1);
#else
        altitude_CTRL = imuData.altitude;
#endif
    }
#endif

    if (altitude_CTRL > control.params.alt_max_sensor)		// Saturation 
        altitude_CTRL = control.params.alt_max_sensor;
    if (altitude_CTRL < control.params.alt_min)
        altitude_CTRL = control.params.alt_min;


    if (control.flags.THROTTLE) {
        // Reset these parameters for safety
        control.altitude_pid.e_old = 0.0;
        control.altitude_pid.u = 0.0;
        control.altitude_pid.i = 0.0;
        if (control.altitude.manual || !control.altitude.enable_auto) {
            if (RCIsReady()) {
                control.altitude.ref = RC_THROTTLE;
            } else {
                // Slowly bring down the throttle
                control.altitude.ref = 0.95 * control.altitude.ref;
                if (altitude_CTRL < 0.1) {
                    // Auto landing
                    RCSMSwitchToIdle();
                    return;
                }
            }
        }
        yaw_u_OP = control.altitude.ref;					 
        // Lower bound to idle speed 
        if (yaw_u_OP < control.params.Kidle) {
            yaw_u_OP = control.params.Kidle;
        }
    } else {
        // Altitude Control ----------------------------------------------------
        if (control.altitude.manual || !control.altitude.enable_auto) {
            control.flags.ABS_ALT_CTRL = 0;
            if (RCIsReady()) {
                control.altitude.ref = RC_THROTTLE * control.params.alt_max;
            } else {
                control.altitude.ref = altitude_CTRL - 0.005;
                if (altitude_CTRL < 0.1) {
                    // Auto landing
                    RCSMSwitchToIdle();
                    return;
                }
            }
        } 

        if (control.altitude.ref > control.params.alt_max)								// Saturation
            control.altitude.ref = control.params.alt_max;
        if (control.altitude.ref < control.params.alt_min)
            control.altitude.ref = control.params.alt_min;

        control.altitude_pid.e = control.altitude.ref - altitude_CTRL;			// CTRl-error
        control.altitude_pid.e = CONTROL_filter(control.altitude_pid.e, 
                control.altitude_pid.e_old, control.params.T_ctrl_s,0.10);

        //ramp error to avoid massive overshoot and saturation of actuators
        CONTROL_ramp(&control.altitude_pid, 0.31);
        // PID control
        CONTROL_pid_ctrl_alt(&control.altitude_pid, control.params.T_ctrl_s);

        //ARWP alt
        CONTROL_arwp_sati(&control.altitude_pid, -0.07, 0.07);		// limits the i part to a given threshold

        control.altitude_pid.u = control.altitude_pid.u + control.params.Kh;				// kh = constant to compensate for the helicopter mass // 

#ifdef VOLTAGE_COMPENSATION
        //scale motor setpoint according to battery voltage
        control.altitude_pid.u = control.altitude_pid.u * (12500.0 / control.batteryvoltage);
#endif

        if (control.altitude_pid.u > (1.0))		// Saturation 
            control.altitude_pid.u = 1.0;
        if (control.altitude_pid.u < 0.0)				
            control.altitude_pid.u = 0.0;

        yaw_u_OP = control.altitude_pid.u;					 
    }


    // Yaw Control ----------------------------------------------------
    if (control.yaw.manual || !control.yaw.enable_auto)
    {
        float rcyaw = -RC_YAW;
        if ((rcyaw > -0.15) && (rcyaw < 0.15)) {
            rcyaw = 0.0;
        }

        control.yaw.ref = rcyaw*YAW_SCALING;									// Max speed setpoint for the yaw = +-1 rad/sec
    } 

    gyro_CTRL = imuData.gyro[2];

    if (gyro_CTRL > 300.0)									// Saturation to max bandwidth of the sensor in [deg/sec]
        gyro_CTRL = 300.0;
    if (gyro_CTRL < -300.0)
        gyro_CTRL = -300.0;

    gyro_CTRL = deg2rad(gyro_CTRL);						// Conversion to [rad]

    //Calculate CTR_Errors			
    control.yaw_pid.e = control.yaw.ref - gyro_CTRL;							// gyro ctrl error

    // PID control
    CONTROL_pid_ctrl_yaw(&control.yaw_pid, control.params.T_ctrl_s);

    //ARWP alt
    CONTROL_arwp_sati(&control.yaw_pid, -0.12, 0.12);		// limits the i part to a given threshold

#ifdef VOLTAGE_COMPENSATION
    //scale motor setpoint according to battery voltage
    control.yaw_pid.u = control.yaw_pid.u * (12500.0 / control.batteryvoltage);
#endif

    if (control.yaw_pid.u > 0.4)		// Saturation 
        control.yaw_pid.u = 0.4;
    if (control.yaw_pid.u < -0.4)				
        control.yaw_pid.u = -0.4;


#ifdef MOUSE_SENSOR
    // Velocity Control ----------------------------------------------------

    // X Velocity Control
    if (control.pitch.manual || !control.pitch.enable_auto){
        if (control.pitch.velocity && mouse.sensor_available){
            //we are in auto velocity control mode
            control.pitch.ref = -RC_PITCH*PITCH_SCALING*1.0;			// Max speed setpoint for the control.velocity_x_pid = +-1 m/sec
            if(control.pitch.ref < 0.15 && control.pitch.ref > -0.15){
                control.pitch.ref = 0;	//set small controlls to zero for proper hovering
            }
        }
        else{
            control.pitch.ref = -(RC_PITCH + RC_PITCH_TRIM*0.2) * PITCH_SCALING;
        }
    }

    // Y Velocity Control
    if (control.roll.manual || !control.roll.enable_auto)
    {
        if(control.roll.velocity && mouse.sensor_available){
            control.roll.ref = RC_ROLL* ROLL_SCALING*1.0;
            if(control.roll.ref < 0.15 && control.roll.ref > -0.15){
                control.roll.ref = 0;	//set small controlls to zero for hover
            }
        }
        else{
            control.roll.ref = (RC_ROLL + RC_ROLL_TRIM*0.2)* ROLL_SCALING;
        }
    }

    //Check if MouseSensor is getting sensible data
    if (mouse.proper_measurement) {
        control.velocity_x_pid.e = control.pitch.ref - mouse.gyrofilt_delta_x;
        control.velocity_y_pid.e = control.roll.ref - mouse.gyrofilt_delta_y;// gyro ctrl error
    }
    else {
        control.velocity_x_pid.e = 0.0; // control.pitch.ref;
        control.velocity_y_pid.e = 0.0; // control.roll.ref;
    }

    // PID control
    CONTROL_pid_ctrl(&control.velocity_x_pid, control.params.T_ctrl_s);
    CONTROL_pid_ctrl(&control.velocity_y_pid, control.params.T_ctrl_s);

    CONTROL_arwp_sati(&control.velocity_x_pid, -0.5, 0.5);		// limits the i part to a given threshold
    CONTROL_arwp_sati(&control.velocity_y_pid, -0.5, 0.5);		// limits the i part to a given threshold

    // Saturation of the output to prevent crashes
    if (control.velocity_y_pid.u > 0.5)
        control.velocity_y_pid.u = 0.5;
    if (control.velocity_y_pid.u < -0.5)
        control.velocity_y_pid.u = -0.5;

    if (control.velocity_x_pid.u > 0.5)
        control.velocity_x_pid.u = 0.5;
    if (control.velocity_x_pid.u < -0.5)
        control.velocity_x_pid.u = -0.5;

    if(mouse.proper_measurement==0)
    {
        if ((control.flags.CONTROL_MODE & 0xF0) != CONTROL_AUTO) {
            //set control to normal roll/pitch input if no measurement
            control.velocity_y_pid.u = (RC_ROLL + RC_ROLL_TRIM*0.2) * ROLL_SCALING;
            control.velocity_x_pid.u = -(RC_PITCH + RC_PITCH_TRIM*0.2) * PITCH_SCALING;
        } else {
            //set control to zero if no measurement
            control.velocity_y_pid.u = (0.0 + RC_ROLL_TRIM*0.2) * ROLL_SCALING;
            control.velocity_x_pid.u = -(0.0 + RC_PITCH_TRIM*0.2) * PITCH_SCALING;
        }
    }

    //--------------------
    //Set Servos
    //--------------------

    if(mouse.sensor_available) {
        if (!control.pitch.velocity && !control.roll.velocity){
            if (((control.flags.CONTROL_MODE & 0xF0) != CONTROL_AUTO) 
                    && !control.flags.COUPLED) {
                mouse_light_set(0);//to save power, but only if we're not in auto
            }
            control.velocity_x_pid.i=0;
            control.velocity_y_pid.i=0;
            control_set_servo_decoupled(control.roll.ref,control.pitch.ref);
        } else if(control.pitch.velocity && !control.roll.velocity) {
            control.velocity_x_pid.i=0;
            control_set_servo_decoupled(control.velocity_y_pid.u,control.pitch.ref);
        } else if(!control.pitch.velocity && control.roll.velocity) {
            control.velocity_y_pid.i=0;
            control_set_servo_decoupled(control.roll.ref,control.velocity_x_pid.u);
        } else if(control.pitch.velocity && control.roll.velocity) {
            control_set_servo_decoupled(control.velocity_y_pid.u,control.velocity_x_pid.u);
        }
    } else {
        control.velocity_x_pid.i=0;
        control.velocity_y_pid.i=0;
        // Ref are set correctly if mouse.sensor_available is 0
        MOTOR_SetServoAngle(control.roll.ref,control.pitch.ref);
    }

#else
    // ROLL/PITCH
    if (control.roll.manual || !control.roll.enable_auto) {
        control.roll.ref = (RC_ROLL + RC_ROLL_TRIM*0.2) * ROLL_SCALING;
    }

    if (control.pitch.manual || !control.pitch.enable_auto) {
        control.pitch.ref = -(RC_PITCH + RC_PITCH_TRIM*0.2) * PITCH_SCALING;
    }

    MOTOR_SetServoAngle(control.roll.ref,control.pitch.ref);
#endif

    //--------------------
    //Set Motor
    //--------------------

#ifdef MANUAL_THROTTLE
    yaw_u_OP = RC_THROTTLE;
#endif

    //Set Motorsignal	
#warning Consider distributing the correction on both motor, to avoid loss of lift
    control.motor.up = (yaw_u_OP - yaw_u_OP*control.params.yawOffset) + control.yaw_pid.u;	
    control.motor.dw =  yaw_u_OP;

    if (RCSMGetState() == RC_KILLED) {
        MOTOR_SetSpeed(0.0,0.0);	// Set Motors (motor_up, motor_down)
        resetPIDs();
    } else {
        MOTOR_SetSpeed(control.motor.up,control.motor.dw);	// Set Motors (motor_up, motor_down)
    }

}



/**************************************************************************/
/* FUNCTION:    T4Interrupt                                               */
/* AUTHOR:  	Cedric Pradalier                                       	  */
/* T4 interrupt. Control loop.                  						  */
/**************************************************************************/
void _ISR_AUTOPSV _T4Interrupt(void)
{	
    static unsigned int flagcount = 0;
    static const unsigned int imuEveryTime = 
        IMU_UPDATE_ALT_GYROZ_EULERZ | 
        IMU_UPDATE_ACCEL | IMU_UPDATE_GYRO;
    static const unsigned int imuflags[] = {
        // IMU_UPDATE_EULER,
        // IMU_UPDATE_ACCEL,
        // IMU_UPDATE_GYRO,
        IMU_UPDATE_MAG,
        IMU_UPDATE_PRES,
        // IMU_UPDATE_ALT,  // Is now part of the control package
        // IMU_UPDATE_DIST // not on coax 0005
        IMU_UPDATE_EULER_FILT, // not on coax 0009
        0 // END OF LIST
    };


    // Update the state ----------------------------------------------------

    if (INT_IMU==1)		// Wait for the IMU to (re)boot
    {
        // update one piece of data at a time, to avoid slowing down the
        // control
#ifdef USE_IMU_FFP
        updateIMUData_ffp(imuEveryTime | imuflags[flagcount]);
#else
        updateIMUData(imuEveryTime | imuflags[flagcount]);
#endif
        flagcount ++;
        if (!imuflags[flagcount]) flagcount = 0; 
    } else {
        control.errors.IMU_REBOOTED = 1;
        LED_RED = 1;
    }

#ifdef MOUSE_SENSOR
    if(control.roll.velocity || control.pitch.velocity || control.flags.COUPLED) {
        mouse_adjust_light();
    }
    // mouse_adjust_light();
    mouse_update_data(); //get data from mouse sensor and calculate speed
#endif

    // Update the flags & do control ---------------------------------------

    updateEnableFlags(); // this check the battery level, and may change the control

    if (control.user_function) {
        control.user_function(control.user_context);
    }


    switch (control.flags.CONTROL_MODE & 0x0F) {
        case CONTROL_FLY:
            auto_control();
            break;
        case CONTROL_RAW:
            if ((control.flags.CONTROL_MODE & 0xF0) != CONTROL_AUTO) {
                auto_control();
            } else {
                // The API is setting up the control parameter by itself
                // So we trust it :)
            }
            break;
        case CONTROL_IDLE:
            if (RCSMGetState() == RC_KILLED) {
#ifdef MOUSE_SENSOR
                mouse_light_set(0);
#endif
                MOTOR_SetSpeed(0.,0.);
            } else {
                // when motor_off is 1, we're coming from stop
                // otherwise, we should not stop the motor
                if (motor_off && (control.flags.time_in_idle < control.params.min_time_in_idle/2)) {
                    // Start one motor at a time...
                    MOTOR_SetSpeed(control.params.Kidle,0.0);		// IDLE speed
                } else {
                    MOTOR_SetSpeed(control.params.Kidle,control.params.Kidle);		// IDLE speed
                    motor_off = 0;
                }

            }
            MOTOR_SetServoAngle(0.,0.);
            break;
        case CONTROL_STOP:
        default:
#ifdef MOUSE_SENSOR
            mouse_light_set(0);
#endif
            motor_off = 1;
            MOTOR_SetSpeed(0.,0.);
            MOTOR_SetServoAngle(0.,0.);
            break;
    }

    IFS1bits.T4IF	= 0;	// Timer4 Interrupt Flag Status off	
}

