/*************************************************************
*
* Low-level controller with API server for the Skybotix AG
* helicopters, and particularly COAX
*
* Developed by:
*  * Cedric Pradalier: cedric.pradalier@skybotix.ch
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
*************************************************************/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include <com/sbconst.h>
#include <com/sbmessage.h>
#include <com/sbstate.h>
#include <com/sbchannel.h>


#include "globals.h"
#include "state.h"
#include "timeouts.h"


#ifdef USE_HARDWARE
#include <p33FJ256GP506.h>
#include <libpic30.h>
/* ===== CUSTOM MADE INCLUDES. ========================================== */

#include <configs/coax_ports.h>
#include <configs/init_port.h>
#include <utils/utils.h>
#include <led/coax_led.h>
#include <configs/coax_config.h>
#include <spi/spi.h>
#include <i2c/e_I2C_protocol.h>
#include <us/sonar.h>
#include <control/control.h>
#include <sharp/sharp.h>
#include <uart/coax_uart.h>
#include <analog/analog.h>
#include <rc/coax_rc.h>
#include <rc/rc_statemachine.h>
#include <agenda/e_agenda.h>
#include <motor/motor.h>
#include <imu/imu.h>
#include <imu/imu.h>
#include <mouse/mouse.h>

// Locals. Ultrasonic --------------------------------------------------
#define LED_ORNG_FOR_STATE

#else 

#define CTRLLOOP_PERIOD_MS 20

#endif


#define ERR(code,message) {if (comm->error) comm->error(SB_COMMLOOP##code,message);}
#define DBG(message) {if (comm->error) comm->error(SB_COMMLOOP_DEBUG,message);}


/*************** Globals *********************/
SBHeliStateRaw heliState;

void buildCapacities(unsigned short capacity[2])
{
	capacity[0] = 0;
	capacity[1] = 0;
	sbAddOneContent(capacity,SBC_MODES);
	sbAddOneContent(capacity,SBC_TIMESTAMP);
	sbAddOneContent(capacity,SBC_RPY);
	sbAddOneContent(capacity,SBC_GYRO);
	sbAddOneContent(capacity,SBC_ACCEL);
	sbAddOneContent(capacity,SBC_MAGNETO);
	sbAddOneContent(capacity,SBC_PRESSURE);
	sbAddOneContent(capacity,SBC_TIMEOUT);
	sbAddOneContent(capacity,SBC_ALTITUDE);
	sbAddOneContent(capacity,SBC_HRANGES);
	sbAddOneContent(capacity,SBC_BATTERY);
	sbAddOneContent(capacity,SBC_CHANNELS);
	sbAddOneContent(capacity,SBC_O_ATTITUDE);
	sbAddOneContent(capacity,SBC_O_ALTITUDE);
	sbAddOneContent(capacity,SBC_O_XY);
	sbAddOneContent(capacity,SBC_O_TOL);
	sbAddOneContent(capacity,SBC_XY_REL);
	sbAddOneContent(capacity,SBC_COAXSPEED);
}


/******************* State managenment *******************/


void fillHeliState()
{
#ifdef USE_HARDWARE
	heliState.errorFlags = 0;
    if (control.errors.IMU_REBOOTED) heliState.errorFlags |= SB_FLAG_IMUCRASH;
    if (control.errors.LOW_POWER_DETECTED) heliState.errorFlags |= SB_FLAG_LOWPOWER;
    if (RCSMGetState() == RC_KILLED) heliState.errorFlags |= SB_FLAG_KILLED;
    if (RCSMGetState() != RC_AUTO) heliState.errorFlags |= SB_FLAG_MANUAL;
    if (!RCIsReady()) heliState.errorFlags |= SB_FLAG_RCLOST;

	heliState.timeStamp = gcomm.timecount; 
	heliState.controlTimeout = gcomm.timeout_ctrl; 
	heliState.watchdogTimeout = gcomm.timeout_cmd; 
	// TODO: check the order here
#if 0
	heliState.roll = (int)(imuData.euler[0]*10.);
	heliState.pitch = (int)(imuData.euler[1]*10.);
	heliState.yaw = (int)(imuData.euler[2]*10.);
#else
	heliState.roll = (int)(imuData.euler_filt[0]*10.);
	heliState.pitch = (int)(imuData.euler_filt[1]*10.);
	heliState.yaw = (int)(imuData.euler_filt[2]*10.);
#endif
	// TODO: that the gyros are velocities
#if 0
	// Using incorrectly scaled imu, but exporting correctly
	heliState.gyro[0] = (int)(imuData.gyro[0]*10.*1024./60);
	heliState.gyro[1] = (int)(imuData.gyro[1]*10.*1024./60);
	heliState.gyro[2] = (int)(imuData.gyro[2]*10.*1024./60);
#else
	heliState.gyro[0] = (int)(imuData.gyro[0]*10.);
	heliState.gyro[1] = (int)(imuData.gyro[1]*10.);
	heliState.gyro[2] = (int)(imuData.gyro[2]*10.);
#endif
	heliState.accel[0] = (int)(imuData.accel[0]*1000);
	heliState.accel[1] = (int)(imuData.accel[1]*1000);
	heliState.accel[2] = (int)(imuData.accel[2]*1000);
	heliState.magneto[0] = (int)(imuData.mag[0]*1000);
	heliState.magneto[1] = (int)(imuData.mag[1]*1000);
	heliState.magneto[2] = (int)(imuData.mag[2]*1000);
	heliState.imutemp = 0;
	heliState.zrange = (int)(imuData.altitude*1000);
	heliState.zfiltered = (int)(imuData.abs_altitude*1000);
	// heliState.zrange = (int)(imuData.dist_bottom*1000);
	heliState.pressure = (int)(imuData.pressure);

#if 0
	updateSONAR_FRONT();
	heliState.hranges[SB_RANGE_FRONT] = (int)(sonar.front*1000);
	updateSONAR_LEFT();
	heliState.hranges[SB_RANGE_LEFT] = (int)(sonar.left*1000);
	updateSONAR_RIGHT();
	heliState.hranges[SB_RANGE_RIGHT] = (int)(sonar.right*1000);
#endif
#if 0
	updateSHARP_FRONT();
	heliState.hranges[SB_RANGE_FRONT] = (int)(sharp.front*1000);
	updateSHARP_LEFT();
	heliState.hranges[SB_RANGE_LEFT] = (int)(sharp.left*1000);
	updateSHARP_RIGHT();
	heliState.hranges[SB_RANGE_RIGHT] = (int)(sharp.right*1000);
	updateSHARP_BACK();
	heliState.hranges[SB_RANGE_BACK] = (int)(sharp.back*1000);
#else
	updateSHARP_FRONT();
	heliState.hranges[SB_RANGE_FRONT] = (int)(sharp_int.front);
	updateSHARP_LEFT();
	heliState.hranges[SB_RANGE_LEFT] = (int)(sharp_int.left);
	updateSHARP_RIGHT();
	heliState.hranges[SB_RANGE_RIGHT] = (int)(sharp_int.right);
	updateSHARP_BACK();
	heliState.hranges[SB_RANGE_BACK] = (int)(sharp_int.back);
#endif
#if 0
	heliState.hranges[SB_RANGE_FRONT] = (int)(imuData.dist_front*1000);
	heliState.hranges[SB_RANGE_LEFT] = (int)(imuData.dist_left*1000);
	heliState.hranges[SB_RANGE_RIGHT] = (int)(imuData.dist_right*1000);
	heliState.hranges[SB_RANGE_BACK] = -1;
#endif

	updateBatteryVoltage(); // This is done in the control function now
	heliState.battery = (int)(voltage_level_filt * 1000);

#if 1
	heliState.o_attitude[0] = (int)(control.roll.ref * 1000);
	heliState.o_attitude[1] = (int)(control.pitch.ref * 1000);
	heliState.o_attitude[2] = (int)(control.motor.up * 1000);
	heliState.o_altitude = (int)(control.motor.dw * 1000);
#else
	// DEBUG
	heliState.o_attitude[0] = (int)(control.yaw_pid.e * 1000);
	heliState.o_attitude[1] = (int)(control.yaw_pid.i * 1000);
	heliState.o_attitude[2] = (int)(control.yaw_pid.u * 1000);
	heliState.o_altitude = (int)(control.yaw.ref * 1000);
#endif
    heliState.coaxspeed.state = 
        (mouse.sensor_available?COAXSPEED_AVAILABLE:0) | 
        (mouse.proper_measurement?COAXSPEED_VALID_MEASUREMENT:0);
    if (mouse.sensor_available) {
        heliState.coaxspeed.light = mouse.light_intensity;
        heliState.coaxspeed.vel_x = (int)(mouse.gyrofilt_delta_x*1000);
        heliState.coaxspeed.vel_y = (int)(mouse.gyrofilt_delta_y*1000);
    } else {
        heliState.coaxspeed.light = 0;
        heliState.coaxspeed.vel_x = 0;
        heliState.coaxspeed.vel_y = 0;
    }

	// unused
	heliState.xrel = (int)(mouse.filt_delta_x*1000);
	heliState.yrel = (int)(mouse.filt_delta_y*1000);
	heliState.o_tol = mouse.data.squal;
	heliState.o_xy[0] = mouse.data.delta_x;
	heliState.o_xy[1] = mouse.data.delta_y;
	heliState.o_oavoid[0] = 0xDEAD;
	heliState.o_oavoid[1] = 0x1234;

	heliState.rcChannel[SB_RC_THROTTLE] = (signed short)(1000.*RC_THROTTLE);
	heliState.rcChannel[SB_RC_THROTTLE_TRIM] = (signed short)(1000.*RC_THROTTLE_TRIM);
	heliState.rcChannel[SB_RC_YAW] = (signed short)(1000.*RC_YAW);
	heliState.rcChannel[SB_RC_YAW_TRIM] = (signed short)(1000.*RC_YAW_TRIM);
	heliState.rcChannel[SB_RC_ROLL] = (signed short)(1000.*RC_ROLL);
	heliState.rcChannel[SB_RC_ROLL_TRIM] = (signed short)(1000.*RC_ROLL_TRIM);
	heliState.rcChannel[SB_RC_PITCH] = (signed short)(1000.*RC_PITCH);
	heliState.rcChannel[SB_RC_PITCH_TRIM] = (signed short)(1000.*RC_PITCH_TRIM);
#else
	heliState.errorFlags=0;
	heliState.timeStamp = gcomm.timecount; 
	heliState.controlTimeout = gcomm.timeout_ctrl; 
	heliState.watchdogTimeout = gcomm.timeout_cmd; 
	heliState.gyro[0] = 0;
	heliState.gyro[1] = 0;
	heliState.gyro[2] = 0;
	heliState.accel[0] = 0;
	heliState.accel[1] = 0;
	heliState.accel[2] = 0;
	heliState.magneto[0] = 0;
	heliState.magneto[1] = 0;
	heliState.magneto[2] = 0;
	heliState.imutemp = 0;
	heliState.zrange = 0;
	heliState.pressure = 0;
	heliState.hranges[0] = 0;
	heliState.hranges[1] = 0;
	heliState.hranges[2] = 0;
	heliState.battery = 0xFEDC;
	heliState.o_attitude[0] = 0xAABB;
	heliState.o_attitude[1] = 0xCCDD;
	heliState.o_attitude[2] = 0xEEFF;
	heliState.o_altitude = 0xDEAD;
	heliState.o_tol = 0xDEAD;
	heliState.o_xy[0] = 0xDEAD;
	heliState.o_xy[1] = 0xDEAD;
	heliState.o_oavoid[0] = 0xDEAD;
	heliState.o_oavoid[1] = 0x1234;
	memset(heliState.rcChannel,0,8*sizeof(signed short));
#endif
}



void state_update_0() {
	// We only send if we finished sending the previous one
	if (!gcomm.stateReady[0]) {
		if (gcomm.stateReady[1]) {
			// the state has been serialised already, but not sent. We don't
			// need to redo it.
			gcomm.stateReady[0] = 1;
		} else if (sbStateEncode(&gcomm.serialisedState,0,&heliState)) {
			ERR(_ERROR, "Failed to encode heli state 0\n");
		} else {
#ifdef LED_ORNG_FOR_STATE
			LED_ORNG = 1 - LED_ORNG;
#endif
			gcomm.stateReady[0] = 1;
		}
	}
}

void state_update_1() {
	// We only send if we finished sending the previous one
	if (!gcomm.stateReady[1]) {
		if (gcomm.stateReady[0]) {
			// the state has been serialised already, but not sent. We don't
			// need to redo it.
			gcomm.stateReady[1] = 1;
		} else if (sbStateEncode(&gcomm.serialisedState,0,&heliState)) {
			ERR(_ERROR, "Failed to encode heli state 1\n");
		} else {
#ifdef LED_ORNG_FOR_STATE
			LED_ORNG = 1 - LED_ORNG;
#endif
			gcomm.stateReady[1] = 1;
		}
	}
}

void setPeriodicState(unsigned int source, unsigned int period_ms) {
	if (period_ms < CTRLLOOP_PERIOD_MS) {
		// There is no point is going faster, the data is only updated at this
		// rate
		period_ms = CTRLLOOP_PERIOD_MS;
	}
	switch (source) {
		case 0:
			e_set_agenda_cycle(state_update_0,period_ms*10);
#ifdef LED_ORNG_FOR_STATE
			if (period_ms == 0) LED_ORNG = 0;
#endif
			break;
		case 1:
			e_set_agenda_cycle(state_update_1,period_ms*10);
#ifdef LED_ORNG_FOR_STATE
			if (period_ms == 0) LED_ORNG = 0;
#endif
			break;
		default:
			ERR(_ERROR, "Invalid source for setPeriodicState\n");
			return;
	}
}


