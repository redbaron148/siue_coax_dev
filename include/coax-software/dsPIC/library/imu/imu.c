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

#include "i2c/e_I2C_protocol.h"
#include "imu/imu.h"
#include "utils/utils.h"


//======================================================
//=== READ byte from the IMU as an external RAM
//======================================================

char read_byte_imu(char reg_address)
{
    char data=0;								// Initialize the data
	data = e_i2cp_read(COAX_IMU_ADDRESS, reg_address);
	return(data);
}


//======================================================
//=== WRITE byte to the IMU as an external RAM
//======================================================

char write_byte_imu(char reg_address, char data)
{
 	char error=0;								// Initialize to no error	
	error = e_i2cp_write (COAX_IMU_ADDRESS, reg_address, data);
	return error;
}

//======================================================
//=== Validate checksum from IMU Packet
//======================================================

int checksum_is_valid(unsigned char *read_buffer, unsigned int len)
{
	register int i;
	unsigned int counter = 0;
	if (len == 0) return -2;
	for (i=1;i<len;i++) {
		counter += read_buffer[i];
	}
	counter = counter & 0xFF;
	return (counter != 0)?-1:0;
}


//======================================================
//=== READ imu version from the IMU as an external RAM
//======================================================
char read_imu_version(char version[8])
{
	char read_buffer[16];
	char error;
	int i;
	error = e_i2cp_read_string(COAX_IMU_ADDRESS, read_buffer, 
			COAX_IMU_VERSION_ADDR, LENGTH_IMU_VERSION);
	if (error) return error;
	error = checksum_is_valid((unsigned char*)read_buffer,LENGTH_IMU_VERSION);
	if (error) return error;
	for (i=0;i<8;i++) {
		version[i] = read_buffer[1+i];
	}
	return error;
}


//======================================================
//=== READ EULER angles from the IMU as an external RAM
//======================================================

char read_euler_imu(char *read_buffer)
{
	char error=0;	
	error = e_i2cp_read_string(COAX_IMU_ADDRESS, read_buffer, COAX_IMU_EULER_LEN, LENGTH_EULER_PACKET);
	if (error) return error;
	error = checksum_is_valid((unsigned char*)read_buffer,LENGTH_EULER_PACKET);
	if (error) return error;
	return 0;
}

char read_euler_imu_filt(char *read_buffer)
{
	char error=0;	
	error = e_i2cp_read_string(COAX_IMU_ADDRESS, read_buffer, COAX_IMU_EULER_FILT_LEN, LENGTH_EULER_FILT_PACKET);
	if (error) return error;
	error = checksum_is_valid((unsigned char*)read_buffer,LENGTH_EULER_FILT_PACKET);
	if (error) return error;
	return 0;
}

//==========================================================
//=== READ LIN ACCELERATIONS from the IMU as an external RAM
//==========================================================

char read_accel_imu(char *read_buffer)
{
	char error=0;	
	error = e_i2cp_read_string(COAX_IMU_ADDRESS, read_buffer, COAX_IMU_ACCEL_LEN, LENGTH_ACCEL_PACKET);
	if (error) return -10 + error;
	error = checksum_is_valid((unsigned char*)read_buffer,LENGTH_ACCEL_PACKET);
	if (error) return -50 + error;
	return 0;
}

//==========================================================
//=== READ GYRO from the IMU as an external RAM
//==========================================================

char read_gyro_imu(char *read_buffer)
{
	char error=0;	
	error = e_i2cp_read_string(COAX_IMU_ADDRESS, read_buffer, COAX_IMU_GYRO_LEN, LENGTH_GYRO_PACKET);
	if (error) return error;
	error = checksum_is_valid((unsigned char*)read_buffer,LENGTH_GYRO_PACKET);
	if (error) return error;
	return 0;
}

//==========================================================
//=== READ MAG from the IMU as an external RAM
//==========================================================

char read_mag_imu(char *read_buffer)
{
	char error=0;	
	error = e_i2cp_read_string(COAX_IMU_ADDRESS, read_buffer, COAX_IMU_MAG_LEN, LENGTH_MAG_PACKET);
	if (error) return error;
	error = checksum_is_valid((unsigned char*)read_buffer,LENGTH_MAG_PACKET);
	if (error) return error;
	return 0;
}


//==========================================================
//=== READ PRESSURE from the IMU as an external RAM
//==========================================================

char read_pressure_imu(char *read_buffer)
{
	char error=0;	
	error = e_i2cp_read_string(COAX_IMU_ADDRESS, read_buffer, COAX_IMU_PRESSURE_LEN, LENGTH_PRESSURE_PACKET);
	if (error) return error;
	error = checksum_is_valid((unsigned char*)read_buffer,LENGTH_PRESSURE_PACKET);
	if (error) return error;
	return 0;
}


//==========================================================
//=== READ ALT from the IMU as an external RAM
//==========================================================

char read_altitude_imu(char *read_buffer)
{
	char error=0;	
	error = e_i2cp_read_string(COAX_IMU_ADDRESS, read_buffer, COAX_IMU_ALTITUDE_LEN, LENGTH_ALTITUDE_PACKET);
	if (error) return error;
	error = checksum_is_valid((unsigned char*)read_buffer,LENGTH_ALTITUDE_PACKET);
	if (error) return error;
	return 0;
}

//==========================================================
//=== READ RAW from the IMU as an external RAM
//==========================================================

char read_raw_imu(char *read_buffer)
{
	char error=0;	
	error = e_i2cp_read_string(COAX_IMU_ADDRESS, read_buffer, COAX_IMU_RAW_LEN, LENGTH_RAW_PACKET);
	if (error) return -10 + error;
	error = checksum_is_valid((unsigned char*)read_buffer,LENGTH_RAW_PACKET);
	if (error) return -50 + error;
	return 0;
}


//==========================================================
//=== READ MININAL DATA FOR COAX CONTROL (Alt, GyroZ, EulerZ)
//==========================================================

char read_alt_gyroz_eulerz_imu(char *read_buffer)
{
	char error=0;	
	error = e_i2cp_read_string(COAX_IMU_ADDRESS, read_buffer, COAX_IMU_ALT_GYROZ_EULERZ_LEN, LENGTH_ALT_GYROZ_EULERZ_PACKET);
	if (error) return error;
	error = checksum_is_valid((unsigned char*)read_buffer,LENGTH_ALT_GYROZ_EULERZ_PACKET);
	if (error) return error;
	return 0;
}


//==========================================================
//=== READ DISTANCE SENSORS
//==========================================================

char read_dist_imu(char *read_buffer)
{
	char error=0;	
	error = e_i2cp_read_string(COAX_IMU_ADDRESS, read_buffer, COAX_IMU_DIST_LEN, LENGTH_DIST_PACKET);
	if (error) return error;
	error = checksum_is_valid((unsigned char*)read_buffer,LENGTH_DIST_PACKET);
	if (error) return error;
	return 0;
}


//================================================================================
//=== READ byte by byte EULER angles from the IMU as an external RAM (OLD NOT USED)
//=================================================================================

void read_slow_euler_imu(char *buf_euler)
{
	*buf_euler   = read_byte_imu(COAX_IMU_EULER_LEN);				// read the 

	*(++buf_euler) = read_byte_imu(COAX_IMU_ROLL_LSB1);		// read the 
	*(++buf_euler) = read_byte_imu(COAX_IMU_ROLL_LSB2);		// read the 
	*(++buf_euler) = read_byte_imu(COAX_IMU_ROLL_MSB1);		// read the 
	*(++buf_euler) = read_byte_imu(COAX_IMU_ROLL_MSB2);		// read the 

	*(++buf_euler) = read_byte_imu(COAX_IMU_PITCH_LSB1);		// read the 
	*(++buf_euler) = read_byte_imu(COAX_IMU_PITCH_LSB2);		// read the 
	*(++buf_euler) = read_byte_imu(COAX_IMU_PITCH_MSB1);		// read the 
	*(++buf_euler) = read_byte_imu(COAX_IMU_PITCH_MSB2);		// read the

	*(++buf_euler) = read_byte_imu(COAX_IMU_YAW_LSB1);		// read the 
	*(++buf_euler) = read_byte_imu(COAX_IMU_YAW_LSB2);		// read the 
	*(++buf_euler) = read_byte_imu(COAX_IMU_YAW_MSB1);		// read the 
	*(++buf_euler) = read_byte_imu(COAX_IMU_YAW_MSB2);		// read the

	*(++buf_euler) = read_byte_imu(COAX_IMU_EULER_CHECKSUM);		// read the
}


//=============================================================
// FUNCTION PROTOTYPE for CALIBRATED DATA
//=============================================================

IMU_DATA imuData;

int updateIMUData(int selectionFlag)
{
	unsigned char test_buf[200]; // !!!!!
	int err = 0;

	if (selectionFlag & IMU_UPDATE_EULER) {
		err = err | read_euler_imu((char*)test_buf);		// GOOD ONE!!

		imuData.euler[0] = convert2float(test_buf[4], test_buf[3], test_buf[2], test_buf[1]);
		imuData.euler[1] = convert2float(test_buf[8], test_buf[7], test_buf[6], test_buf[5]);
		imuData.euler[2] = convert2float(test_buf[12], test_buf[11], test_buf[10], test_buf[9]);
	}

	if (selectionFlag & IMU_UPDATE_EULER_FILT) {
		err = err | read_euler_imu_filt((char*)test_buf);		// GOOD ONE!!

		imuData.euler_filt[0] = convert2float(test_buf[4], test_buf[3], test_buf[2], test_buf[1]);
		imuData.euler_filt[1] = convert2float(test_buf[8], test_buf[7], test_buf[6], test_buf[5]);
		imuData.euler_filt[2] = convert2float(test_buf[12], test_buf[11], test_buf[10], test_buf[9]);
	}


	if (selectionFlag & IMU_UPDATE_ACCEL) {
		err = err | read_accel_imu((char*)test_buf);
		imuData.accel[0] = convert2float(test_buf[4], test_buf[3], test_buf[2], test_buf[1]);
		imuData.accel[1] = convert2float(test_buf[8], test_buf[7], test_buf[6], test_buf[5]);
		imuData.accel[2] = convert2float(test_buf[12], test_buf[11], test_buf[10], test_buf[9]);
	}

	if (selectionFlag & IMU_UPDATE_GYRO) {
		err = err | read_gyro_imu((char*)test_buf);
		// WARNING: check deg/rad conversion
		imuData.gyro[0] = convert2float(test_buf[4], test_buf[3], test_buf[2], test_buf[1]);
		imuData.gyro[1] = convert2float(test_buf[8], test_buf[7], test_buf[6], test_buf[5]);
		imuData.gyro[2] = convert2float(test_buf[12], test_buf[11], test_buf[10], test_buf[9]);
	}

	if (selectionFlag & IMU_UPDATE_ALT_GYROZ_EULERZ) {
		err = err | read_alt_gyroz_eulerz_imu((char*)test_buf);
		imuData.altitude = convert2float(test_buf[4], test_buf[3], test_buf[2], test_buf[1]);
		imuData.gyro[2]  = convert2float(test_buf[8], test_buf[7], test_buf[6], test_buf[5]);
		imuData.euler_filt[2] = convert2float(test_buf[12], test_buf[11], test_buf[10], test_buf[9]);
	}

	if (selectionFlag & IMU_UPDATE_MAG) {
		err = err | read_mag_imu((char*)test_buf);
		imuData.mag[0] = convert2float(test_buf[4], test_buf[3], test_buf[2], test_buf[1]);
		imuData.mag[1] = convert2float(test_buf[8], test_buf[7], test_buf[6], test_buf[5]);
		imuData.mag[2] = convert2float(test_buf[12], test_buf[11], test_buf[10], test_buf[9]);
	}

	if (selectionFlag & IMU_UPDATE_ALT) {					
		err = err | read_altitude_imu((char*)test_buf);
		imuData.altitude = convert2float(test_buf[4], test_buf[3], test_buf[2], test_buf[1]);
	}

	if (selectionFlag & IMU_UPDATE_PRES) {
		err = err | read_pressure_imu((char*)test_buf);
		imuData.pressure = convert2float(test_buf[4], test_buf[3], test_buf[2], test_buf[1]);
	}

	if (selectionFlag & IMU_UPDATE_DIST) {
		err = err | read_dist_imu((char*)test_buf);
		imuData.dist_bottom = convert2float(test_buf[4], test_buf[3], test_buf[2], test_buf[1]);
		imuData.dist_front = convert2float(test_buf[8], test_buf[7], test_buf[6], test_buf[5]);
		imuData.dist_left = convert2float(test_buf[12], test_buf[11], test_buf[10], test_buf[9]);
		imuData.dist_right = convert2float(test_buf[16], test_buf[15], test_buf[14], test_buf[13]);
	}


	return err;
}

int getIMURaw(unsigned char * i2cbuffer, unsigned short * buffer) 
{
	unsigned char *ptr = i2cbuffer+1;
	unsigned int i, numshort = 0;
	int err;
	err = read_mag_imu((char*)i2cbuffer);
	if (err) return -200 + err;

	if (i2cbuffer[0] < 2) {
		return 0;
	}
	numshort = (i2cbuffer[0] - 2)>>1;
	for (i=0;i<numshort;i++) {
		register unsigned short s1,s2;
		s1 = *ptr; ptr++;
		s2 = *ptr; ptr++;
		buffer[i] = s1 | (s2 << 8);
	}

	return numshort;
}







