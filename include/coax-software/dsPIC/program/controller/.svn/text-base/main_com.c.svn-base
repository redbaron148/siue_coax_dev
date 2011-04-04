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

//#include <p30f6014A.h>
#include <p33FJ256GP506.h>
#include <libpic30.h>

#include <string.h>
#include <ctype.h>
#include <stdio.h>

/* ===== CUSTOM MADE INCLUDES. ========================================== */

#include <configs/coax_config.h>
#include <configs/coax_ports.h>
#include <configs/init_port.h>
#include <utils/utils.h>
#include <led/coax_led.h>
#include <configs/coax_config.h>
#include <spi/spi.h>
#include <i2c/e_I2C_protocol.h>
#include <us/altitude.h>
#include <us/sonar.h>
#include <uart/coax_uart.h>
#include <analog/analog.h>
#include <rc/coax_rc.h>
#include <agenda/e_agenda.h>
#include <motor/motor.h>
#include <imu/imu.h>
#include <imu/imu_ffp.h>
#include <control/control.h>
#include <mouse/mouse.h>


#include <com/sbconst.h>
#include <com/sbchannel.h>
#include <com/sbcommloop.h>

#include "globals.h"
#include "control.h"
#include "timeouts.h"
#include "state.h"
#include "version.h"
#include "bluetooth.h"

#define MOUSE_SENSOR

#ifdef MOUSE_SENSOR
#include "mouse/mouse.h"
#endif

#ifdef COAX003X
#include "control-coax003x.h"
#endif

#ifdef COAX002X
#include "control-coax002x.h"
#endif

/* ===== CONFIGURATION BITS ============================================= */


_FBS(BWRP_WRPROTECT_OFF);							// Boot segment
_FSS(SSS_NO_SEC_CODE & SSS_NO_FLASH & SWRP_WRPROTECT_OFF);		// Secure segment

_FOSCSEL(FNOSC_PRI);								// Primary (XT, HS, EC) Oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF  & POSCMD_HS);  
// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: XT Crystanl
_FWDT(FWDTEN_OFF & WINDIS_OFF)						// Watchdog Timer
_FPOR(FPWRT_PWR128);								// POR (Power On Reset)


/* ===== DEFINITIONS. =================================================== */





/* ===== GLOBALS. ======================================================= */

/* ===== Utilities. ========================================================== */


/* ===== MAIN. ========================================================== */


void errorPIC33(unsigned int errorLevel, const char * message)
{
	switch (errorLevel) {
		case SB_COMMLOOP_CRITICAL:
			DEBUG(message);
			LED_RED = 1;
#ifdef USE_HARDWARE
			BuzzerBip(-1,0);		// Make x bips of the Buzzer
#endif
			break;
		case SB_COMMLOOP_ERROR:
			DEBUG(message);
			LED_RED = 1;
#ifdef USE_HARDWARE
			BuzzerBip(1,0);		// Make x bips of the Buzzer
#endif
			break;
		case SB_COMMLOOP_WARNING:
			DEBUG(message);
			break;
		case SB_COMMLOOP_DEBUG:
			if (comm->verbose) {DEBUG(message);}
			break;
		case SB_COMMLOOP_DEBUG2:
			if (comm->verbose>1) {DEBUG(message);}
			break;
		case SB_COMMLOOP_OK:
		default:
			break;
	}
}

void resetPIC33()
{
	Reset();
}

// Get/Set control parameters, must return 0 on success, -1 if settings are
// ignored
int updateControlParams(int setParams, SBControlParametersMessage *params)
{
	if (setParams) {
		control.params.yawOffset = params->yawOffset/1000.;
		control.params.Kh = params->baseThrust/1000.;
		control.yaw_pid.kp = params->yawKp/1000.;
		control.yaw_pid.ki = params->yawKi/1000.;
		control.yaw_pid.kd = params->yawKd/1000.;
		control.altitude_pid.kp = params->altitudeKp/1000.;
		control.altitude_pid.ki = params->altitudeKi/1000.;
		control.altitude_pid.kd = params->altitudeKd/1000.;
	} else {
		params->yawOffset = (int)(control.params.yawOffset*1000);
		params->baseThrust = (int)(control.params.Kh*1000);
		params->altitudeKp = (int)(control.altitude_pid.kp*1000);
		params->altitudeKi = (int)(control.altitude_pid.ki*1000);
		params->altitudeKd = (int)(control.altitude_pid.kd*1000);
		params->yawKp = (int)(control.yaw_pid.kp*1000);
		params->yawKi = (int)(control.yaw_pid.ki*1000);
		params->yawKd = (int)(control.yaw_pid.kd*1000);
	}
	return 0;
}

// Get/Set trim mode, must return 0 on success, -1 if settings are
// ignored
int updateTrimMode(int setMode, SBTrimModeMessage *mode)
{
	if (setMode) {
		setSoftwareTrim(mode->trimMode,
				mode->rollTrim/10000.,mode->pitchTrim/10000.);
	} else {
		mode->trimMode = getTrimMode();
		mode->rollTrim = (int)(getTrimRoll(mode->trimMode)*10000);
		mode->pitchTrim = (int)(getTrimPitch(mode->trimMode)*10000);
	}
	return 0;
}

// Configure the bluetooth access code and name
int configureBluetoothPIC33(const char code[4], const char name[16])
{
	LED_RED = 1;
	if(e_bt_write_local_pin_number(code)) {
		errorPIC33(SB_COMMLOOP_ERROR,"Error writting PIN\n\r");
		return -1;
	}

	if(e_bt_write_local_name(name)) {
		errorPIC33(SB_COMMLOOP_ERROR,"Error writting Name\n\r");
		return -2;
	}
	LED_RED = 0;
	return 0;
}

int main(void) {

	char imu_board_id[9] = {0,0,0,0, 0,0,0,0, 0};
	// Init Oscillator. ----------------------------------------------------

	InitOscillator();			// Initialize the PLL (also disables wdt)
	__delay32(_50MILLISEC);


	// Init mcu ports ------------------------------------------------------

	init_port();    	// Initialize ports

#ifdef USE_HARDWARE
	// Init MOTORS & SERVOS. ---------------------------------------------------------

	MOTOR_InitMotorControl();				// Must be at the beginnig to ensure proper init of MC.
	MOTOR_SetSpeed(0.0, 0.0);				// Must be at the beginnig to ensure proper init of MC.
	MOTOR_SetServoAngle(0.0, 0.0);				// Must be at the beginnig to ensure proper init of MC.

	// Init Analog Channels. -----------------------------------------------

	analog_initAnalog();	// Init the ADC module

	// Init SPI. ---------------------------------------------------------

	init_SPI();

	// Init I2C. ---------------------------------------------------------

	e_i2cp_init();
	e_i2cp_enable();
	__delay32(_50MILLISEC);
	e_i2c_write(0x00);	// dummy byte to get the START bit on the bus (believe it or not!)

	// Init ctrl. --------------------------------------------------------
#ifdef COAX003X
#define CONTROL_INITIALISED
	initControlVariables(initCoax003xControlParameters);
#endif
#ifdef COAX002X
#define CONTROL_INITIALISED
	initControlVariables(initCoax002xControlParameters);
#endif
#ifndef CONTROL_INITIALISED
#error Control not initialised
#endif

	BuzzerBip(1,1);		// Make x bips of the Buzzer (blocking)


	// Init RC. ----------------------------------------------------------

	RCInitReception(0);

#ifdef RC2401
#define RCSELECTED
    RCSetType(RC_WK2401);
#endif
#ifdef RC2402
#define RCSELECTED
    RCSetType(RC_WK2402);
#endif
#ifndef RCSELECTED
#error Control not initialised
#endif



	// Init LED. ----------------------------------------------------------

	LED_ORNG =1;
	LED_RED = 0;


#endif
	__delay32(_200MILLISEC);  // Wait for the IMU to boot

	while (INT_IMU==0)		// Wait for the IMU to boot
	{
		FlashORNG ();						// Flash the LED	
	}
	read_imu_version(imu_board_id); imu_board_id[8] = 0;
	sbSetIMUVersion(imu_board_id);
	
	// Init BUZZER. ----------------------------------------------------------

	BuzzerBip(3,1);		// Make x bips of the Buzzer, blocking
    LED_ORNG = 0;
	LED_RED = 1;


	InitLoopTimer(control.params.T_ctrl_ms);// Initialize & Enable control loop timer (20ms; 50Hz)


	// Init AGENDA. ---------------------------------------------------------
	e_start_agendas_processing();
	// Init LED. ----------------------------------------------------------
	// flash(4,500);
	//

	// Will be called in sbStateMachine (from control function)
	sbCommLoopInit(&gcomm,CONTROLLER_VERSION);
	gcomm.updateHeliState = fillHeliState;
	gcomm.setPeriodicState = setPeriodicState;
	gcomm.error = errorPIC33;
	gcomm.reset = resetPIC33;
	gcomm.configureBluetooth = configureBluetoothPIC33;
	gcomm.updateControlParams = updateControlParams;
	gcomm.updateTrimMode = updateTrimMode;
#ifdef MOUSE_SENSOR
    gcomm.setLight = mouse_light_set;
#endif
	gcomm.verbose = 0;
	// gcomm.debug_channel = CHANNEL_UART_ID;
	// gcomm.debug_channel = CHANNEL_BT_ID;
	gcomm.debug_channel = -1;
	gcomm.takeoffHeight_mm = 0;
	gcomm.landingHeight_mm = 180;
	gcomm.landingStep_mm = 5;
	buildCapacities(gcomm.capacities); // initialise the list of capacities

	// Insert periodic functions in the agenda
	// This is used in all the function that are non-blocking to compute
	// their time-out.
	e_activate_agenda(timecount_increment,10);
	e_activate_agenda(state_update_0,0); // this will be set once the client is requesting it
	e_activate_agenda(state_update_1,0); // this will be set once the client is requesting it

	// e_activate_agenda(control_function,control.params.T_ctrl_ms*10); // the set points can be updated only every 20ms
	// Put the control function in the control interrupt user function.
	// It will also be responsible for updating the time counter.
	control.user_context = NULL;
	control.user_function = control_function;


	// Channels are defined in the global variables
	sbChannelCreateCoaxUART1(gcomm.channel+CHANNEL_UART_ID);
	sbChannelOpen(gcomm.channel+CHANNEL_UART_ID);
	sbChannelSendString(gcomm.channel+CHANNEL_UART_ID,"\n\n\rHello, I'm happy (SER "__DATE__")\n",500);

	sbChannelCreateCoaxUART2(gcomm.channel+CHANNEL_BT_ID);
	sbChannelOpen(gcomm.channel+CHANNEL_BT_ID);
	sbChannelSendString(gcomm.channel+CHANNEL_BT_ID,"\n\n\rHello, I'm happy (BT "__DATE__")\n",500);
	DEBUG("Debug function is activated on this channel\n");
#if 1
	if (imu_board_id[0] == 0) {
		// Wrong IMU, or imu not initialised
		errorPIC33(SB_COMMLOOP_CRITICAL, "IMU not initialised");
	} else {
		char message[64];
		sprintf(message, "IMU Ready: board %8s\n",imu_board_id);
		DEBUG(message); 
	}
#endif
	LED_RED = 0;
#ifdef MOUSE_SENSOR
	// Init Mouse Sensor. ----------------------------------------------------------
	mouse_init();
    if (mouse.sensor_available) {
        DEBUG("CoaxEye sensor ready\n"); 
    } else {
        DEBUG("CoaxEye sensor not detected\n"); 
    }
	//mouse_send_image();//to send the image over bluetooth (stays in the function)
#endif

#if 1
	sbCommLoop(&gcomm,&heliState);
#else
	while (1) {}
#endif
	
	sbChannelDestroy(gcomm.channel+CHANNEL_UART_ID);
	sbChannelDestroy(gcomm.channel+CHANNEL_BT_ID);
	return 0;
};



//====================================================
//====    START OF INTERRUPT SERVICE ROUTINES    =====
//====================================================

// TIInterrupt is declared in the coax_rc library

// T4Interrupt is declared in the control library


//=============================================================================
//Error traps
//=============================================================================

//====================================
//Oscillator Fail Error trap routine
//====================================

void _ISR_AUTOPSV _OscillatorFail(void)
{
	INTCON1bits.OSCFAIL=0;		// fail flag clearing
	DEBUG("\nOscillator Error\n\n");
	__asm__ volatile ("reset");
}

//====================================
//Address Error trap routine
//====================================

void _ISR_AUTOPSV _AddressError(void) 	
{	
	INTCON1bits.ADDRERR=0;		// fail flag clearing
	DEBUG("\nAddress Error\n\n");
	__asm__ volatile ("reset");
}

//====================================
//Stack Error trap routine
//====================================

void _ISR_AUTOPSV _StackError(void)		
{		
	INTCON1bits.STKERR=0;		// fail flag clearing
	DEBUG("\nStack Error\n\n");
	__asm__ volatile ("reset");
}

//====================================
//Math (Arithmetic) Error trap routine
//====================================

void _ISR_AUTOPSV _MathError(void)		
{	
	INTCON1bits.MATHERR=0;		// fail flag clearing
	DEBUG("\nMath Error\n\n");
	__asm__ volatile ("reset");
}

