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
* Description:This software is a I2C based driver for the CoaX IMU.
*************************************************************/

#ifndef _IMU_H
#define _IMU_H



//========================================================
// LENGTH OF IMU DATA PACKETS (also coded in IMU software)
//========================================================


#define LENGTH_EULER_PACKET					14	// 1x Length + 12x data + 1x checksum
#define LENGTH_EULER_FILT_PACKET			14	// 1x Length + 12x data + 1x checksum
#define LENGTH_ACCEL_PACKET					14
#define LENGTH_GYRO_PACKET					14
#define LENGTH_MAG_PACKET					14
#define LENGTH_PRESSURE_PACKET				6
#define LENGTH_ALTITUDE_PACKET				6
#define LENGTH_RAW_PACKET	    			20
#define LENGTH_ALT_GYROZ_EULERZ_PACKET	    14
#define LENGTH_DIST_PACKET					18
#define LENGTH_IMU_VERSION					10  // 1x + 8 bytes descr + checksum

//=============================================================
// ADDRESSE OF IMU SLAVE MODULE
//=============================================================

#define COAX_IMU_ADDRESS		0x10	// Address of the IMU slave module
#define COAX_IMU_VERSION_ADDR	116


//====================================================
// ADDRESSES OF FILTERED EULER ANGLES REGISTERS IN THE COAX IMU
//====================================================

#define COAX_IMU_EULER_FILT_LEN		0x00		// Adress of data packet length

#define COAX_IMU_ROLL_FILT_LSB1		0x01
#define COAX_IMU_ROLL_FILT_LSB2		0x02
#define COAX_IMU_ROLL_FILT_MSB1		0x03
#define COAX_IMU_ROLL_FILT_MSB2		0x04

#define COAX_IMU_PITCH_FILT_LSB1		0x05
#define COAX_IMU_PITCH_FILT_LSB2		0x06
#define COAX_IMU_PITCH_FILT_MSB1		0x07
#define COAX_IMU_PITCH_FILT_MSB2		0x08

#define COAX_IMU_YAW_FILT_LSB1		0x09
#define COAX_IMU_YAW_FILT_LSB2		0x0A
#define COAX_IMU_YAW_FILT_MSB1		0x0B
#define COAX_IMU_YAW_FILT_MSB2		0x0C

#define COAX_IMU_EULER_FILT_CHECKSUM	0x0D

//====================================================
// ADDRESSES OF EULER ANGLES REGISTERS IN THE COAX IMU
//====================================================

#define COAX_IMU_EULER_LEN		0x0e		// Adress of data packet length

#define COAX_IMU_ROLL_LSB1		0x0f
#define COAX_IMU_ROLL_LSB2		0x10
#define COAX_IMU_ROLL_MSB1		0x11
#define COAX_IMU_ROLL_MSB2		0x12

#define COAX_IMU_PITCH_LSB1	0x13
#define COAX_IMU_PITCH_LSB2	0x14
#define COAX_IMU_PITCH_MSB1	0x15
#define COAX_IMU_PITCH_MSB2	0x16

#define COAX_IMU_YAW_LSB1		0x17
#define COAX_IMU_YAW_LSB2		0x18
#define COAX_IMU_YAW_MSB1		0x19
#define COAX_IMU_YAW_MSB2		0x1A

#define COAX_IMU_EULER_CHECKSUM	0x1B

//====================================================
// ADDRESSES OF ACCELERATION REGISTERS IN THE COAX IMU
//====================================================

#define COAX_IMU_ACCEL_LEN		0x1C		//  	

#define COAX_IMU_ACCEL_X_LSB1	0x1D
#define COAX_IMU_ACCEL_X_LSB2	0x1e
#define COAX_IMU_ACCEL_X_MSB1	0x1f
#define COAX_IMU_ACCEL_X_MSB2	0x20
#define COAX_IMU_ACCEL_Y_LSB1	0x21
#define COAX_IMU_ACCEL_Y_LSB2	0x22
#define COAX_IMU_ACCEL_Y_MSB1	0x23
#define COAX_IMU_ACCEL_Y_MSB2	0x24
#define COAX_IMU_ACCEL_Z_LSB1	0x25
#define COAX_IMU_ACCEL_Z_LSB2	0x26
#define COAX_IMU_ACCEL_Z_MSB1	0x27
#define COAX_IMU_ACCEL_Z_MSB2	0x28

#define COAX_IMU_ACCEL_CHECKSUM	0x29

//====================================================
// ADDRESSES OF GYRO DATRA REGISTERS IN THE COAX IMU
//====================================================

#define COAX_IMU_GYRO_LEN		0x2A		// 	

#define COAX_IMU_GYRO_X_LSB1	0x2B
#define COAX_IMU_GYRO_X_LSB2	0x2C
#define COAX_IMU_GYRO_X_MSB1	0x2D
#define COAX_IMU_GYRO_X_MSB2	0x2e
#define COAX_IMU_GYRO_Y_LSB1	0x2f
#define COAX_IMU_GYRO_Y_LSB2	0x30
#define COAX_IMU_GYRO_Y_MSB1	0x31
#define COAX_IMU_GYRO_Y_MSB2	0x32
#define COAX_IMU_GYRO_Z_LSB1	0x33
#define COAX_IMU_GYRO_Z_LSB2	0x34
#define COAX_IMU_GYRO_Z_MSB1	0x35
#define COAX_IMU_GYRO_Z_MSB2	0x36

#define COAX_IMU_GYRO_CHECKSUM	0x37

//================================================
// ADDRESSES OF MAGNETO REGISTERS IN THE COAX IMU
//================================================


#define COAX_IMU_MAG_LEN		0x38		//  	

#define COAX_IMU_MAG_X_LSB1		0x39
#define COAX_IMU_MAG_X_LSB2		0x3A
#define COAX_IMU_MAG_X_MSB1		0x3B
#define COAX_IMU_MAG_X_MSB2		0x3C
#define COAX_IMU_MAG_Y_LSB1		0x3D
#define COAX_IMU_MAG_Y_LSB2		0x3e
#define COAX_IMU_MAG_Y_MSB1		0x3f
#define COAX_IMU_MAG_Y_MSB2		0x40
#define COAX_IMU_MAG_Z_LSB1		0x41
#define COAX_IMU_MAG_Z_LSB2		0x42
#define COAX_IMU_MAG_Z_MSB1		0x43
#define COAX_IMU_MAG_Z_MSB2		0x44

#define COAX_IMU_MAG_CHECKSUM	0x45

//================================================
// ADDRESSES OF PRESSURE REGISTERS IN THE COAX IMU
//================================================

#define COAX_IMU_PRESSURE_LEN		70		//  	

#define COAX_IMU_PRESSURE_LSB1		71
#define COAX_IMU_PRESSURE_LSB2		72
#define COAX_IMU_PRESSURE_MSB1		73
#define COAX_IMU_PRESSURE_MSB2		74

#define COAX_IMU_PRESSURE_CHECKSUM	75

//================================================
// ADDRESSES OF ALTITUDE REGISTERS IN THE COAX IMU
//================================================

#define COAX_IMU_ALTITUDE_LEN		76	

#define COAX_IMU_ALTITUDE_LSB1		77
#define COAX_IMU_ALTITUDE_LSB2		78
#define COAX_IMU_ALTITUDE_MSB1		79
#define COAX_IMU_ALTITUDE_MSB2		80

#define COAX_IMU_ALTITUDE_CHECKSUM	67

//================================================
// ADDRESSES OF RMININAL DATA FOR COAX CONTROL (Alt, GyroZ, EulerZ)
//================================================

#define COAX_IMU_ALT_GYROZ_EULERZ_LEN		82	

//================================================
// ADDRESSES OF RAW SENS REGISTERS IN THE COAX IMU
//================================================

#define COAX_IMU_RAW_LEN		96		// 		CHANGED !!!!


//================================================
// ADDRESSES OF DISTANCE
//================================================

#define COAX_IMU_DIST_LEN		102	
#define COAX_IMU_DIST_CHECKSUM	119

//=============================================================
// IMU DATA TYPE
//=============================================================

typedef struct {
	float euler[3];
	float euler_filt[3];
	float accel[3];
	float rawZgyro;
	float altitude;
	float abs_altitude;
	float ground_altitude;
	float gyro[3];
	float gyro_old[3];
	float mag[3];
	float pressure;
	float dist_bottom;
	float dist_front;
	float dist_left;
	float dist_right;
	float quaternion[4];
	float gyro_bias[3];
} IMU_DATA;

//=============================================================
// FUNCTION PROTOTYPE for CALIBRATED DATA
//=============================================================

extern IMU_DATA imuData;

#define IMU_UPDATE_EULER 			0x0001
#define IMU_UPDATE_ACCEL 			0x0002
#define IMU_UPDATE_GYRO  			0x0004
#define IMU_UPDATE_ALT_GYROZ_EULERZ	0x0008
#define IMU_UPDATE_MAG   			0x0010
#define IMU_UPDATE_ALT   			0x0020
#define IMU_UPDATE_PRES   			0x0040
#define IMU_UPDATE_DIST				0x0080
#define IMU_UPDATE_EULER_FILT		0x0100
#define IMU_UPDATE_QUATERNION		0x0200
#define IMU_UPDATE_GYRO_BIAS		0x0400
#define IMU_UPDATE_ALL   			0x01FF
int updateIMUData(int selectionFlag);

// This buffer should contain at least 12 words
int getIMURaw(unsigned char * i2cbuffer, unsigned short *buffer);


//=============================================================
// FUNCTION PROTOTYPES for RAW ACCESS
//=============================================================

//char read_byte_imu(unsigned short reg_address);
//char write_byte_imu(unsigned short reg_address, char data);
char read_byte_imu(char reg_address);
char write_byte_imu(char reg_address, char data);
char read_euler_imu(char *read_buffer);
char read_accel_imu(char *read_buffer);
char read_gyro_imu(char *read_buffer);
char read_mag_imu(char *read_buffer);
char read_altitude_imu(char *read_buffer);
char read_pressure_imu(char *read_buffer);
char read_raw_imu(char *read_buffer);
char read_alt_gyroz_eulerz_imu(char *read_buffer);
char read_dist_imu(char *read_buffer);
char read_imu_version(char version[8]);

void read_slow_euler_imu(char *buf_euler);

#endif
