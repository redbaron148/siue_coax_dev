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

#include "i2c/e_I2C_protocol.h"
#include "imu/imu_ffp.h"
#include "utils/utils.h"

unsigned short IMU_RCON = 0;

//======================================================
//=== Validate checksum from IMU Packet
//======================================================

static 
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


static
char read_packet(unsigned char *buffer, unsigned int addr, unsigned int len)
{
	char error=0;	
	error = e_i2cp_read_string(COAX_IMU_ADDRESS, (char*)buffer, 
			COAX_FFP_IMU_OFFSET + addr, len);
	if (error) return 0x80 | error;
	error = checksum_is_valid(buffer,len);
	if (error) return 0x40 | error;
	return 0;
}


//======================================================
//=== READ EULER angles from the IMU as an external RAM
//======================================================

#define UC2SHORT(a,b) ((signed short)((((unsigned short)(b))<<8) | (a)))

static 
long uc2long(unsigned char * c) {
	unsigned long l = c[3];
	l = (l<<8) + c[2];
	l = (l<<8) + c[1];
	l = (l<<8) + c[0];
	return l;
}


char read_ffp_euler_imu(float euler[3], int filtered)
{
#if LENGTH_FFP_EULER_PACKET != LENGTH_FFP_EULER_FILT_PACKET
#error Inconsistent length for euler packets
#endif
	unsigned char buffer[LENGTH_FFP_EULER_PACKET];
	char error=0;	
	if (filtered) {
		error = read_packet(buffer,COAX_FFP_IMU_EULER_FILT_ADDR,LENGTH_FFP_EULER_FILT_PACKET);
	} else {
		error = read_packet(buffer,COAX_FFP_IMU_EULER_ADDR,LENGTH_FFP_EULER_PACKET);
	}
	if (error) return error;
	euler[0] = UC2SHORT(buffer[1],buffer[2]) * (180./0x8000);
	euler[1] = UC2SHORT(buffer[3],buffer[4]) * (180./0x8000);
	euler[2] = UC2SHORT(buffer[5],buffer[6]) * (180./0x8000);
	return 0;
}

//==========================================================
//=== READ LIN ACCELERATIONS from the IMU as an external RAM
//==========================================================

char read_ffp_accel_imu(float accel[3])
{
	char error=0;	
	unsigned char buffer[LENGTH_FFP_ACCEL_PACKET];
	error = read_packet(buffer,COAX_FFP_IMU_ACCEL_ADDR,LENGTH_FFP_ACCEL_PACKET);
	if (error) return error;
	accel[0] = UC2SHORT(buffer[1],buffer[2]) / 32.;
	accel[1] = UC2SHORT(buffer[3],buffer[4]) / 32.;
	accel[2] = UC2SHORT(buffer[5],buffer[6]) / 32.;
	return 0;
}

//==========================================================
//=== READ GYRO from the IMU as an external RAM
//==========================================================

char read_ffp_gyro_imu(float gyro[3])
{
	char error=0;	
	unsigned char buffer[LENGTH_FFP_GYRO_PACKET];
	error = read_packet(buffer,COAX_FFP_IMU_GYRO_ADDR,LENGTH_FFP_GYRO_PACKET);
	if (error) return error;
	gyro[0] = UC2SHORT(buffer[1],buffer[2])  * (720./(0x8000));
	gyro[1] = UC2SHORT(buffer[3],buffer[4])  * (720./(0x8000));
	// warning, read also in read_ffp_alt_gyroz_eulerz_imu
	gyro[2] = UC2SHORT(buffer[5],buffer[6])  * (720./(0x8000));
	return 0;
}

//==========================================================
//=== READ MAG from the IMU as an external RAM
//==========================================================

char read_ffp_mag_imu(float mag[3])
{
	char error=0;	
	unsigned char buffer[LENGTH_FFP_MAG_PACKET];
	error = read_packet(buffer,COAX_FFP_IMU_MAG_ADDR,LENGTH_FFP_MAG_PACKET);
	if (error) return error;
	mag[0] = UC2SHORT(buffer[1],buffer[2]) / ((float)0x8000);
	mag[1] = UC2SHORT(buffer[3],buffer[4]) / ((float)0x8000);
	mag[2] = UC2SHORT(buffer[5],buffer[6]) / ((float)0x8000);
	return 0;
}

char read_ffp_quaternion_imu(float q[4])
{
	char error=0;	
	unsigned char buffer[LENGTH_FFP_ACT_QUATERNION];
	error = read_packet(buffer,COAX_FFP_IMU_ACT_QUATERNION,LENGTH_FFP_ACT_QUATERNION);
	if (error) return error;
	q[0] = uc2long(buffer+1) / ((float)0x8000);
	q[1] = uc2long(buffer+5) / ((float)0x8000);
	q[2] = uc2long(buffer+9) / ((float)0x8000);
	q[3] = uc2long(buffer+13) / ((float)0x8000);
	return 0;
}

char read_ffp_gyrobias_imu(float b[3])
{
	char error=0;	
	unsigned char buffer[LENGTH_FFP_ACT_BIAS];
	error = read_packet(buffer,COAX_FFP_IMU_ACT_BIAS,LENGTH_FFP_ACT_BIAS);
	if (error) return error;
	b[0] = uc2long(buffer+1) * 180. / ((float)0x8000) / ((float)(1<<14));
	b[1] = uc2long(buffer+5) * 180. / ((float)0x8000) / ((float)(1<<14));
	b[2] = uc2long(buffer+9) * 180. / ((float)0x8000) / ((float)(1<<14));
	return 0;
}


//==========================================================
//=== READ PRESSURE from the IMU as an external RAM
//==========================================================

char read_ffp_pressure_imu(float pressure[1])
{
	char error=0;	
	unsigned char buffer[LENGTH_FFP_PRESSURE_PACKET];
	error = read_packet(buffer,COAX_FFP_IMU_PRESSURE_ADDR,LENGTH_FFP_PRESSURE_PACKET);
	if (error) return error;
	pressure[0] = UC2SHORT(buffer[1],buffer[2]);
	return 0;
}


//==========================================================
//=== READ ALT from the IMU as an external RAM
//==========================================================

char read_ffp_altitude_imu(float altitude[3])
{
	char error=0;	
	unsigned char buffer[LENGTH_FFP_ALTITUDE_PACKET];
	error = read_packet(buffer,COAX_FFP_IMU_ALTITUDE_ADDR,LENGTH_FFP_ALTITUDE_PACKET);
	if (error) return error;
	altitude[0] = UC2SHORT(buffer[1],buffer[2]) / 1024.; // raw measurement
	altitude[1] = UC2SHORT(buffer[3],buffer[4]) / 1024.; // absolute altitude
	altitude[2] = UC2SHORT(buffer[5],buffer[6]) / 1024.; // ground estimate
	return 0;
}


//==========================================================
//=== READ MININAL DATA FOR COAX CONTROL (Alt, GyroZ, EulerZ)
//==========================================================

char read_ffp_alt_gyroz_eulerz_imu(float alt[1], float gyroz[1], float abs_alt[1])
{
	char error=0;	
	unsigned char buffer[LENGTH_FFP_ALT_GYROZ_EULERZ_PACKET];
	error = read_packet(buffer,COAX_FFP_IMU_ALT_GYROZ_EULERZ_ADDR,
			LENGTH_FFP_ALT_GYROZ_EULERZ_PACKET);
	if (error) return error;
	alt[0] = UC2SHORT(buffer[1],buffer[2]) / 1024.;
	gyroz[0] = UC2SHORT(buffer[3],buffer[4])  * (720./0x8000);
	//eulerz[0] = UC2SHORT(buffer[5],buffer[6]) * (180./0x8000);
	abs_alt[0] = UC2SHORT(buffer[5],buffer[6]) / 1024.;
	return 0;
}


//=============================================================
// FUNCTION PROTOTYPE for CALIBRATED DATA
//=============================================================

int updateIMUData_ffp(int selectionFlag)
{
	int err = 0;

	if (selectionFlag & IMU_UPDATE_EULER) {
		err = err | read_ffp_euler_imu(imuData.euler,0);
	}

	if (selectionFlag & IMU_UPDATE_EULER_FILT) {
		err = err | read_ffp_euler_imu(imuData.euler_filt,1);
	}

	if (selectionFlag & IMU_UPDATE_ACCEL) {
		err = err | read_ffp_accel_imu(imuData.accel);
	}

	if (selectionFlag & IMU_UPDATE_GYRO) {
		err = err | read_ffp_gyro_imu(imuData.gyro);
	}

	if (selectionFlag & IMU_UPDATE_ALT_GYROZ_EULERZ) {
		err = err | read_ffp_alt_gyroz_eulerz_imu(&imuData.altitude,
				imuData.gyro+2, &imuData.abs_altitude);
	}

	if (selectionFlag & IMU_UPDATE_MAG) {
		err = err | read_ffp_mag_imu(imuData.mag);
	}

	if (selectionFlag & IMU_UPDATE_ALT) {					
		float altitude[3];
		err = err | read_ffp_altitude_imu(altitude);
		imuData.altitude = altitude[0];
		imuData.abs_altitude = altitude[1];
		imuData.ground_altitude = altitude[2];
	}

	if (selectionFlag & IMU_UPDATE_PRES) {
		err = err | read_ffp_pressure_imu(&imuData.pressure);
	}

	if (selectionFlag & IMU_UPDATE_QUATERNION) {
		err = err | read_ffp_quaternion_imu(imuData.quaternion);
	}

	if (selectionFlag & IMU_UPDATE_GYRO_BIAS) {
		err = err | read_ffp_gyrobias_imu(imuData.gyro_bias);
	}

	return err;
}


