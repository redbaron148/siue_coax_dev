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

#ifndef __UTILS_H_
#define __UTILS_H_

#include <configs/coax_ports.h>


void WaitMiliSec(unsigned int mils);
void WaitMicroSec(unsigned int mics);
// Perform bip beeps (continuously if -1). If not blocking, use the agenda timer.
void BuzzerBip(unsigned char bip,int blocking);

float absolute(float var);
void Buzzer_ON(unsigned int frequency);
void Buzzer_OFF(void);

extern unsigned int voltage_level_int;
extern float voltage_level;
extern float voltage_level_filt;

typedef enum {
	BATTERY_VOLTAGE_OK,
	BATTERY_VOLTAGE_LOW,
	BATTERY_VOLTAGE_CRITICAL
} BatteryVoltageEnum;

void updateBatteryVoltage(void);
BatteryVoltageEnum evalBatteryVoltageLEDAlert(void);

unsigned int median5(unsigned int x1,unsigned int x2,unsigned int x3,unsigned int x4,unsigned int x5);
unsigned int median3(unsigned int x1,unsigned int x2,unsigned int x3);
float median3_float(float x1,float x2,float x3);
//void BinToAsciiString(char *destBuf, char *srcBuf, int size);
//char BinToChar(char bin);

unsigned char uint_lsb(unsigned short);
unsigned char uint_msb(unsigned short);
float convert2float(unsigned char msb2, unsigned  char msb1, unsigned char lsb2, unsigned char lsb1);

float rad2deg(float);
float deg2rad(float);

#endif


							
	
