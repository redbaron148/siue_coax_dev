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
* Description:This software is a I2C based driver for the CoaX IMU (fixed point
* version
*************************************************************/

#ifndef _IMU_FFP_H
#define _IMU_FFP_H

#include "imu.h"


//========================================================
// LENGTH OF IMU DATA PACKETS (also coded in IMU software)
//========================================================


#define LENGTH_FFP_EULER_PACKET					8	// 1x Length + 6 data + 1x checksum
#define LENGTH_FFP_EULER_FILT_PACKET			8	// 1x Length + 6 data + 1x checksum
#define LENGTH_FFP_ACCEL_PACKET					8
#define LENGTH_FFP_GYRO_PACKET					8
#define LENGTH_FFP_MAG_PACKET					8
#define LENGTH_FFP_PRESSURE_PACKET				4
#define LENGTH_FFP_ALTITUDE_PACKET				8
#define LENGTH_FFP_ALT_GYROZ_EULERZ_PACKET	    8
#define LENGTH_FFP_ACT_QUATERNION			   18
#define LENGTH_FFP_ACT_BIAS					   14


//====================================================
// ADDRESSES OF EULER ANGLES REGISTERS IN THE COAX IMU
//====================================================


#define COAX_FFP_IMU_OFFSET		0x80		// Beginning of FFP specific packets
#define COAX_FFP_IMU_EULER_FILT_ADDR	0		// Adress of data packet length
#define COAX_FFP_IMU_EULER_ADDR		8		// Adress of data packet length

//====================================================
// ADDRESSES OF ACCELERATION REGISTERS IN THE COAX IMU
//====================================================

#define COAX_FFP_IMU_ACCEL_ADDR		16		//  	

//====================================================
// ADDRESSES OF GYRO DATRA REGISTERS IN THE COAX IMU
//====================================================

#define COAX_FFP_IMU_GYRO_ADDR		24		// 	

//================================================
// ADDRESSES OF MAGNETO REGISTERS IN THE COAX IMU
//================================================


#define COAX_FFP_IMU_MAG_ADDR		32		//  	

//================================================
// ADDRESSES OF PRESSURE REGISTERS IN THE COAX IMU
//================================================

#define COAX_FFP_IMU_PRESSURE_ADDR		40//  	

//================================================
// ADDRESSES OF ALTITUDE REGISTERS IN THE COAX IMU
//================================================

#define COAX_FFP_IMU_ALTITUDE_ADDR		44

//================================================
// ADDRESSES OF RMININAL DATA FOR COAX CONTROL (Alt, GyroZ, EulerZ)
//================================================

#define COAX_FFP_IMU_ALT_GYROZ_EULERZ_ADDR		52	

//================================================
// ADDRESSES OF FFP version of the Quaternion
//================================================

#define COAX_FFP_IMU_ACT_QUATERNION		60	

//================================================
// ADDRESSES OF FFP version of the Gyro Bias
//================================================

#define COAX_FFP_IMU_ACT_BIAS		78	

extern unsigned short IMU_RCON;

int updateIMUData_ffp(int selectionFlag);

//=============================================================
// FUNCTION PROTOTYPES for RAW ACCESS
//=============================================================

char read_ffp_euler_imu(float euler[3], int filtered);
char read_ffp_accel_imu(float accel[3]);
char read_ffp_gyro_imu(float gyro[3]);
char read_ffp_mag_imu(float mag[3]);
char read_ffp_altitude_imu(float altitude[3]);
char read_ffp_pressure_imu(float pressure[1]);
char read_ffp_quaternion_imu(float quaternion[4]);
char read_ffp_gyrobias_imu(float bias[3]);

char read_ffp_alt_gyroz_eulerz_imu(float alt[1], float gyroz[1], float eulerz[1]);

#endif // _IMU_FFP_H
