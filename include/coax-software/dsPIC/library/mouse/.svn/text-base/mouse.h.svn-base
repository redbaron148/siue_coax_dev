/*************************************************************
*
* Mouse sensor functions for the COAX
*
* Developed by:
*  * Pascal Gohl: gohlp@ethz.ch
*  * Matthias Egli: eglima@ethz.ch
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

#ifndef _MOUSE_H
#define _MOUSE_H

//================================================
// DECLARATIONS
//================================================

//old factors
//#define CAMERA_FACTOR_Y 342. //Must be float
//#define CAMERA_FACTOR_X 287. //must be float
#define LIGHT_MIN_ALTITUDE 0.01

//Length of Filter for Mouse Data. Must be 2*size, as X and Y need to be saved.
#define MOUSE_FILTER_LENGTH 24


typedef struct {
	char delta_x;
	char delta_y;
	unsigned char squal;
	unsigned short shutter;
	unsigned char max_pixel;
	unsigned char min_pixel;
} MOUSE_DATA;

typedef struct {

	MOUSE_DATA data;

	int sensor_available;
	float filt_delta_x;
	float filt_delta_y;
	float gyrofilt_delta_x;
	float gyrofilt_delta_y;
	int inactive;
	short history[MOUSE_FILTER_LENGTH];
	unsigned int start_of_history;
	int light_intensity;
	int light_dutycycle;
	int proper_measurement;
	int start_of_gyros_history;
	float gyros_history_roll[MOUSE_FILTER_LENGTH];
	float gyros_history_pitch[MOUSE_FILTER_LENGTH];
	float gyros_filt[2];

} MOUSE_STATUS;

extern MOUSE_STATUS mouse;

//=============================================================
// FUNCTION PROTOTYPES: public functions
//=============================================================

//returns 1 if a sensor is connected else 0
void mouse_init();

//reads the sensor data and calculates the velocity
void mouse_update_data();

//resets the mouse sensor
void mouse_reset();

//get the velocity data from the mouse sensor
void mouse_measure();

//sets the mouse light to the value in percent
void mouse_light_set(unsigned short intensity);

//reads only one value
unsigned char mouse_read(unsigned char addr);

//writes data to the sensor
void mouse_write(unsigned char addr, unsigned char data);

//continuously writes the sensor image data to uart2 (bluetooth). Stays in the function!
void mouse_send_image();

//=============================================================
// FUNCTION PROTOTYPES: private functions
//=============================================================

//reads all measured data at one time (faster)
void mouse_read_burst(unsigned char data[7]);

//gets the value of the next pixel (sensor shifts at his own)
unsigned char mouse_read_pixel();

//Filter Code
void mouse_sw_filter();
void mouse_fir_filter();

//calculates and sets the required light
void mouse_adjust_light();

//Filters the angular movement from the mouse sensor data.
//Output:
// mouse: Saves gyro-compensated signal to mouse->gyrofilt_delta_x/y
void mouse_gyro_compensate();

void mouse_gyro_fir_filter();

//private function declared
void mouse_clock();


#endif

