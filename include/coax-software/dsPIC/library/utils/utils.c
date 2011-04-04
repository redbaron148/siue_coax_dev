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
* Description: This software brings some general purpose functions, in
* particular battery and buzzer management
*************************************************************/

#include <p33FJ256GP506.h>
#include <libpic30.h>
#include "utils/utils.h"
#include "configs/coax_config.h"
#include "analog/analog.h"
#include "agenda/e_agenda.h"


unsigned int voltage_level_int=0;
float voltage_level=0;	// battery voltage level
float voltage_level_filt = 12; // initially -1, so we can initialise it at the first reading

/**************************************************************************/
/* FUNCTION:    Buzzer bip      	                                      */
/**************************************************************************/

static int buzzer_agenda_initialised = 0;
static int beeping = 0; /* are we currently beeping */
static unsigned int biptogo = 0; /* the number of bips to go */

static void buzzer_agenda()
{
	BUZZER = 1 - BUZZER;
}


static void bip_agenda()
{
	if (beeping) {
		Buzzer_OFF();
		if (biptogo>0) {
			biptogo --;
		}
		if (!biptogo) {
			e_set_agenda_cycle(bip_agenda,0);
		}
	} else {
        LED_RED = 1;
		Buzzer_ON(500);
	}
}

static void initialise_buzzer_agenda()
{
	if (!buzzer_agenda_initialised) {
		beeping = 0;
		biptogo = 0;
		e_activate_agenda(bip_agenda,0);
		e_activate_agenda(buzzer_agenda,0);
		buzzer_agenda_initialised = 1;
	}
}

void BuzzerBip(unsigned char bip,int blocking)
{
	if (blocking) {
		unsigned int i;
		unsigned char j;
		for(j=bip;j;j--) {
			for(i=1000;i;i--) {				// Buzzer vibrate
				if (BUZZER==0) {
					BUZZER =1;
					__delay32(_200MICROSEC);
				}	else {
					BUZZER =0;
					__delay32(_200MICROSEC);
				}
			}
			__delay32(_100MILLISEC);				// delay between the bips
		}
	} else {
		initialise_buzzer_agenda();
		biptogo = bip; 
		e_set_agenda_cycle(bip_agenda,2500); // bip at 2 hertz
	}
}



/**************************************************************************/
/* FUNCTION:    Buzzer ON for ~1 Minute  CAUTION: Add huge delay!!        */
/**************************************************************************/

void Buzzer_ON(unsigned int frequency)
{
#if 0
	long i;
	for(i=200000;i;i--)	{			// Buzzer vibrate
		if (BUZZER==0) {
			BUZZER =1;
			__delay32(_200MICROSEC);
		}	else {
			BUZZER =0;
			__delay32(_200MICROSEC);
		}
	}
#else
	initialise_buzzer_agenda();
	e_set_agenda_cycle(buzzer_agenda,10000/(2*frequency)); // the A is 240Hz
	beeping = 1;
#endif
}


/**************************************************************************/
/* FUNCTION:    Buzzer OFF 			                                      */
/**************************************************************************/

void Buzzer_OFF(void)
{
	e_set_agenda_cycle(buzzer_agenda,0); // the A is 240Hz
	beeping = 0;
	BUZZER==0;
}

/**************************************************************************/
/* FUNCTION:    Wait functions   	                                      */
/**************************************************************************/
void WaitMiliSec(unsigned int mils)		// TO REMOVE !!!
{
	__delay32(mils * _1MILLISEC);
}

void WaitMicroSec(unsigned int mics)
{
	__delay32(mics * _1MICROSEC);
}


/**************************************************************************/
/* FUNCTION:    Battery voltage reading & alerting if level too low 	                                      */
/**************************************************************************/

void updateBatteryVoltage(void) {
	const float kf = 0.01;
	voltage_level_int = analog_readChannel(0);
	voltage_level = 0.0034*voltage_level_int - 1.5871;
    voltage_level_filt = kf * voltage_level + (1-kf)*voltage_level_filt;
}

BatteryVoltageEnum evalBatteryVoltageLEDAlert()
{
	if (voltage_level_filt > 10.5) {
		LED_RED=0; 
		return BATTERY_VOLTAGE_OK;
	} else if (voltage_level_filt < 10.0) {
		LED_RED=1; 
		BuzzerBip(-1,0);
		return BATTERY_VOLTAGE_CRITICAL;
	} else {
		LED_RED=1; 
		return BATTERY_VOLTAGE_LOW;
	}
}



float convert2float(unsigned char msb2, unsigned  char msb1, unsigned char lsb2, unsigned char lsb1)
{
	float lac[1];
	unsigned char *lac_ptr_char;

	lac_ptr_char = (unsigned char*) lac;
	lac_ptr_char[0]= lsb1;
	lac_ptr_char[1]= lsb2;
	lac_ptr_char[2]= msb1;
	lac_ptr_char[3]= msb2;

	return lac[0];
}

float rad2deg(float angle)
{
	return (angle/3.1415926535898*180);
}

float deg2rad(float angle)
{
	return (angle*3.1415926535898/180);
}

float median3_float(float x1,float x2,float x3)
{
	float median_value;
	if (x1 > x2){
		if (x1 < x3) {
			median_value = x1;
		} else if (x3 > x2) {
			median_value = x3;
		} else {
			median_value = x2;
		}
	} else{
		if (x1 > x3) {
			median_value = x1;
		} else if (x3 < x2) {
			median_value = x3;
		} else {
			median_value = x2;
		}
	}
	return median_value;
}

unsigned int median3(unsigned int x1,unsigned int x2,unsigned int x3)
{
	unsigned int median_value;
	if (x1 > x2){
		if (x1 < x3) {
			median_value = x1;
		} else if (x3 > x2) {
			median_value = x3;
		} else {
			median_value = x2;
		}
	}
	else{
		if (x1 > x3) {
			median_value = x1;
		} else if (x3 < x2) {
			median_value = x3;
		} else {
			median_value = x2;
		}
	}
	return median_value;
}



unsigned int median5(unsigned int x1,unsigned int x2,unsigned int x3,unsigned int x4,unsigned int x5)
{
	unsigned int median_value;
	if ((x1 < x2) && (x1 < x3) && (x1 < x4) && (x1 < x5)){
		if ((x5 > x4) && (x5 > x3) && (x5 > x2)) {
			median_value = median3(x2,x3,x4);
		} else if ((x4 > x5) && (x4 > x3) && (x4 > x2)) {
			median_value = median3(x2,x3,x5);
		} else if ((x3 > x5) && (x3 > x4) && (x3 > x2)) {
			median_value = median3(x2,x4,x5);
		} else {
			median_value = median3(x3,x4,x5);
		}
	} else if ((x2 < x1) && (x2 < x3) && (x2 < x4) && (x2 < x5)){
		if ((x5 > x4) && (x5 > x3) && (x5 > x1)) {
			median_value = median3(x1,x3,x4);
		} else if ((x4 > x5) && (x4 > x3) && (x4 > x1)) {
			median_value = median3(x1,x3,x5);
		} else if ((x3 > x5) && (x3 > x4) && (x3 > x1)) {
			median_value = median3(x1,x4,x5);
		} else {
			median_value = median3(x3,x4,x5);
		}
	} else if ((x3 < x1) && (x3 < x2) && (x3 < x4) && (x3 < x5)){
		if ((x5 > x4) && (x5 > x2) && (x5 > x1)) {
			median_value = median3(x1,x2,x4);
		} else if ((x4 > x5) && (x4 > x2) && (x4 > x1)) {
			median_value = median3(x1,x2,x5);
		} else if ((x2 > x5) && (x2 > x4) && (x2 > x1)) {
			median_value = median3(x1,x4,x5);
		} else {
			median_value = median3(x2,x4,x5);
		}
	} else if ((x4 < x1) && (x4 < x2) && (x4 < x3) && (x4 < x5)){
		if ((x5 > x3) && (x5 > x2) && (x5 > x1)) {
			median_value = median3(x1,x2,x3);
		} else if ((x3 > x5) && (x3 > x2) && (x3 > x1)) {
			median_value = median3(x1,x2,x5);
		} else if ((x2 > x5) && (x2 > x3) && (x2 > x1)) {
			median_value = median3(x1,x3,x5);
		} else {
			median_value = median3(x2,x3,x5);
		}
	}
	else{
		if ((x4 > x3) && (x4 > x2) && (x4 > x1)) {
			median_value = median3(x1,x2,x3);
		} else if ((x3 > x4) && (x3 > x2) && (x3 > x1)) {
			median_value = median3(x1,x2,x4);
		} else if ((x2 > x4) && (x2 > x3) && (x2 > x1)) {
			median_value = median3(x1,x3,x4);
		} else {
			median_value = median3(x2,x3,x4);
		}
	}

	return median_value;
}
//
//
//
float absolute(float var)
{
	float var_temp;
	if (var < 0)
		var_temp = var*(-1);
	else
		var_temp = var;		


	return var_temp;
}	


unsigned char uint_lsb(unsigned short value)
{
	unsigned short temp[1];
	temp[0] = value;
	unsigned char *temp_ptr_char;

	temp_ptr_char = (unsigned char*) temp;
	return temp_ptr_char[0];
}

unsigned char uint_msb(unsigned short value)
{
	unsigned short temp[1];
	temp[0] = value;
	unsigned char *temp_ptr_char;

	temp_ptr_char = (unsigned char*) temp;
	return temp_ptr_char[1];
}

