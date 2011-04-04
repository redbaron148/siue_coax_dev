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
#include <p33FJ256GP506.h>
#include <configs/coax_ports.h>
#include <configs/coax_config.h>
#include <stdio.h>
#include <libpic30.h>
#include "mouse/mouse.h"
#include "imu/imu.h"
#include "uart/coax_uart.h"

//#define USE_OPEN_DRAIN
#define USE_CHIP_SELECT

//pin definitions
#define MOUSE_SCLK	B4
#if 0 
#warning OLD HARDWARE WIRING FOR THE MOUSE SENSOR
// Old hardware
#define MOUSE_CS	D5
#define MOUSE_SDIO	B3
#else
#define MOUSE_CS	B3
#define MOUSE_SDIO	D5
#endif
#define MOUSE_LIGHT	D4

#define T_CLOCK 0.5*MICROSEC
#define T_SRAD 4*MICROSEC

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//macros for port access shortcuts
#ifndef __MACROS_H__
#define __MACROS_H__

#define PORTx(x)	_R ## x
#define LATx(x)		_LAT ## x
#define TRISx(x)	_TRIS ## x
#define ODCx(x)		_ODC ## x

#define PORT(x)	PORTx(x)
#define LAT(x)	LATx(x)
#define TRIS(x)	TRISx(x)		// 0: output, 1: input
#define ODC(x)	ODCx(x)			// 0: normal, 1: open drain

#endif

//factor that converts sensor measurement into meter
#define CAMERA_FACTOR_X 330//better on 30cm443//pcalc:393. //by hand: 389
#define CAMERA_FACTOR_Y 330//better on 30cm391//pcalc:341. //by hand: 202

static int pixel_counter = 0; //for image send

MOUSE_STATUS mouse;

void mouse_init()
{
	//init variables:
	mouse.data.delta_x = 0;
	mouse.data.delta_y = 0;
	mouse.data.max_pixel = 0;
	mouse.data.min_pixel = 0;
	mouse.data.shutter = 0;
	mouse.data.squal = 0;
	mouse.filt_delta_x = 0;
	mouse.filt_delta_y = 0;
	mouse.gyrofilt_delta_x = 0;
	mouse.gyrofilt_delta_y = 0;
	mouse.inactive = 100;
	mouse.start_of_history = 0;
	mouse.light_intensity = 0;
    mouse.proper_measurement=0;

	mouse.start_of_gyros_history = 0;

	//init ports:
	// set as output
#ifdef USE_CHIP_SELECT
	TRIS(MOUSE_CS) = 0;
#endif
	
	TRIS(MOUSE_SCLK) = 0;
	TRIS(MOUSE_SDIO) = 0;
	
#ifdef USE_OPEN_DRAIN
	// put both CLK and SDIO in open drain mode to drive 5V.
	ODC(MOUSE_SCLK) = 1;
	ODC(MOUSE_SDIO) = 1;
#endif
	
	// put clock and chip select to inactive (= high)
#ifdef USE_CHIP_SELECT
	LAT(MOUSE_CS) = 1;	// chip select active low	
#endif

	
	LAT(MOUSE_SCLK) = 1;	// clk active low
	LAT(MOUSE_SDIO) = 1; // put to high to limit current through pull-up
	
	//test if a mouse sensor is connected
	int count = 0;
	int i = 0;
	
	//check if sensor is connected	
	for(i = 0; i<3; i++){
		if( mouse_read(0x00) == 0x12 ) count += 1;
		Nop(); Nop(); Nop(); Nop(); Nop();
		Nop(); Nop(); Nop(); Nop(); Nop();
	}
	if(count == 0){
		//no sensor connected
		mouse.sensor_available = 0;
		return;
	}

	mouse_reset();

	//Set max resolution
	mouse_write(0x19, 0b00011011);


	//init mouse light------------------------------
	// set as output
	TRIS(MOUSE_LIGHT) = 0;

	// put pin in open drain mode to drive 5V FET.
	//ODC(MOUSE_LIGHT) = 1;

	// !!!!!!!!!!! DIRTY HACK!!!!!!!!!!!!!!!!!!!!!!!!!!
	// shortcut betwen pin D6 and D5 on coax008!
//	TRIS(D5) = 1; //pin 5 as input
//	ODC(D5) = 1;
//	LAT(D5) = 1;
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	LAT(MOUSE_LIGHT) = 0;

	// Output Compare 5
	OC5RS		= 450;
	OC5CONbits.OCTSEL = 0; 	// Select Timer 2 as output compare time base
	OC5R = 0; 				// Load the Compare Register Value
	OC5CONbits.OCM = 0b110; // Select the Output Compare mode

	// Initialize and enable Timer2
	T2CONbits.TON = 0; 		// Disable Timer
	T2CONbits.TCS = 0; 		// Select internal instruction cycle clock
	T2CONbits.TGATE = 0; 	// Disable Gated Timer mode
	T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
	TMR2 = 0x00; 			// Clear timer register
	PR2 = 500; 				// Load the period value
	IPC1bits.T2IP = 0x01; 	// Set Timer 2 Interrupt Priority Level
	IFS0bits.T2IF = 0; // Clear Timer 2 Interrupt Flag
	IEC0bits.T2IE = 1; // Enable Timer 2 interrupt
	T2CONbits.TON = 1; // Start Timer

	//ODC(MOUSE_LIGHT) = 1;
	mouse_light_set(0);
	mouse.sensor_available = 1;
}

void mouse_update_data(){

	//adjust the mouse sensor light
	// mouse_adjust_light();

	//update data from the mouse sensor
	mouse_measure();

//debug
//	char msg[20];
//	int lengh = 0;
//	lengh = sprintf(msg,"\n%+2i,",mouse.delta_x);
//	e_send_uart2_char(msg,lengh);


	//Filter Mouse Sensor data to smooth the signal
	mouse_fir_filter();

	mouse_gyro_fir_filter();

	//convert gyros to rad/sec
	//TODO: change mouse algorithm to deg
	mouse.gyros_filt[0] = mouse.gyros_filt[0]/180.*M_PI;
	mouse.gyros_filt[1] = mouse.gyros_filt[1]/180.*M_PI;

	//subtract the measured movement generated from rotation
	mouse_gyro_compensate();
}

void mouse_reset()
{	
	mouse_write(0x3a, 0x5a);
	__delay32(_100MICROSEC);
}


void mouse_clock()
{
	// f_clk = max 3 MHz -> min 0.33 usec
	LAT(MOUSE_SCLK) = 0;
	__delay32(T_CLOCK);
	LAT(MOUSE_SCLK) = 1;
}

void mouse_write(unsigned char addr, unsigned char data)
{	
	LAT(MOUSE_CS) = 0;
	// set data to output
	TRIS(MOUSE_SDIO) = 0;
	Nop();Nop();Nop();Nop();	// we need to wait between direction change and access to pin
	
	// send 1 to indicate write
	LAT(MOUSE_SDIO) = 1;
	mouse_clock();
	
	// send 7 LSb of addr, starting with MSb
	unsigned char i = 7;
	while(i--)
	{
		// 4 more nop to achieve < 3MHz (total 14 operations)
		Nop(); Nop(); Nop(); Nop(); Nop();
		
		LAT(MOUSE_SDIO) = (addr & 0b01000000 ? 1 : 0);
		
		//t_setup = 120ns
		Nop(); Nop(); Nop(); Nop(); Nop();
		
		mouse_clock();
		addr = addr << 1;
	}
	
	// send data
	i = 8;
	while(i--)
	{
		//  wait to achieve < 3MHz
		__delay32(T_CLOCK);
		
		LAT(MOUSE_SDIO) = (data & 0b10000000 ? 1 : 0);
		mouse_clock();
		data = data << 1;
	}
	
	// put SDIO to 1 to limit current through pull up
	LAT(MOUSE_SDIO) = 1;
#ifdef USE_CHIP_SELECT
	LAT(MOUSE_CS) = 1;	// chip select active low
#endif

}

unsigned char mouse_read(unsigned char addr)
{
#ifdef USE_CHIP_SELECT
	LAT(MOUSE_CS) = 0;
#endif
	// set data to output
	TRIS(MOUSE_SDIO) = 0;
	Nop(); Nop(); Nop(); Nop();	// we need to wait between direction change and access to pin
	
	// send 0 to indicate read
	LAT(MOUSE_SDIO) = 0;

	//t_setup = 120ns
	Nop(); Nop(); Nop(); Nop(); Nop();

	mouse_clock();

	// send 7 LSb of addr, starting with MSb
	unsigned char i = 7;
	while(i--)
	{
		// 5 more nop to achieve < 3MHz (total 14 operations)
		Nop(); Nop(); Nop(); Nop(); Nop();
		LAT(MOUSE_SDIO) = (addr & 0b01000000 ? 1 : 0);
		//t_setup = 120ns
		Nop(); Nop(); Nop(); Nop(); Nop();
		mouse_clock();
		addr = addr << 1;
	}
	
	// tSRAD
	__delay32(T_SRAD);
	
	// set data to input
	//ODC(MOUSE_SDIO) = 0;
	TRIS(MOUSE_SDIO) = 1;
	
	// read 8 bits, starting with MSb
	i = 8;
	unsigned char output = 0;
	while(i--)
	{
		__delay32(T_CLOCK);
		
		// clock down
		LAT(MOUSE_SCLK) = 0;
		
		// min tDLY = 120 ns
		__delay32(T_CLOCK);
		
		// read
		output = output << 1;
		if(PORT(MOUSE_SDIO))
			output = output | 1;

		// clock up
		LAT(MOUSE_SCLK) = 1;
	}

	// set as output
	//TRIS(MOUSE_SDIO) = 0;
	//ODC(MOUSE_SDIO) = 1;
	
	// limit current through pull-up
	//LAT(MOUSE_SDIO) = 1;

#ifdef USE_CHIP_SELECT
	LAT(MOUSE_CS) = 1;	// chip select active low
#endif

	return output;
}

void mouse_read_burst(unsigned char data[7])
{
#ifdef USE_CHIP_SELECT
	LAT(MOUSE_CS) = 0;
	// set data to output
	TRIS(MOUSE_SDIO) = 0;
	// send reading of Motion_Burst 0x63
	unsigned char i = 8;
	unsigned addr = 0x63;
	while(i--)
	{
		// 5 more nop to achieve < 3MHz (total 14 operations)
		Nop(); Nop(); Nop(); Nop(); Nop();

		LAT(MOUSE_SDIO) = (addr & 0b10000000 ? 1 : 0);

		// let the signal rise
		Nop(); Nop(); Nop(); Nop(); Nop();

		mouse_clock();
		addr = addr << 1;
	}

	// set data to input
	TRIS(MOUSE_SDIO) = 1;
	
	// read 7 bytes
	unsigned char j;
	for(j = 0; j < 7; j++)
	{
		// read 8 bits, starting with MSb
		int i = 8;
		unsigned char output = 0;
		while(i--)
		{
			__delay32(T_CLOCK);
			
			// clock down
			LAT(MOUSE_SCLK) = 0;
			
			__delay32(T_CLOCK);
			
			// read
			output = output << 1;
			if(PORT(MOUSE_SDIO))
				output = output | 1;
			
			// clock up
			LAT(MOUSE_SCLK) = 1;
		}
		
		data[j] = output;
	}
	
	// limit current through pull-up
	LAT(MOUSE_SDIO) = 1;
	LAT(MOUSE_CS) = 1;	// chip select active low
#else
	//TODO write read for all burst data whidout burst mode
#endif
}

void mouse_adjust_light(){
	//ajusting the light if higher than 5cm
	if(imuData.altitude > LIGHT_MIN_ALTITUDE){
		if(mouse.data.squal < 35) mouse.light_intensity+=1;
		if(mouse.data.squal > 45) mouse.light_intensity-=1;
		if(mouse.light_intensity>100) mouse.light_intensity = 100;
		if(mouse.light_intensity<0) mouse.light_intensity = 0;
	}
	else {
		mouse.light_intensity = 5;
	}

	mouse_light_set(mouse.light_intensity);
}

void mouse_measure()
{
#ifdef USE_CHIP_SELECT
	unsigned char data[7];//debug
	mouse_read_burst(data);

	mouse.data.delta_x = -data[0]; //delta_X
	mouse.data.delta_y = data[1]; //delta_Y
	mouse.data.squal = data[2]; //SQUAL
	mouse.data.shutter = data[3] + (data[4]<<8); //Shutter_upper + shutter_lower (little endian!)
	mouse.data.max_pixel = data[5]; //max pixel
	mouse.data.min_pixel = data[6]; //min pixel
#else
	mouse.data.delta_x = mouse_read(0x03); //delta_X
	mouse.data.delta_y = mouse_read(0x04); //delta_Y
	mouse.data.squal = mouse_read(0x05); //SQUAL
	//mouse.data.shutter = mouse_read(0x03) + (mouse_read(0x03)<<8); //Shutter_upper + shutter_lower (little endian!)
	//mouse.data.max_pixel = mouse_read(0x03); //max pixel
	//mouse.data.min_pixel = mouse_read(0x03); //min pixel
#endif
	//check for inactivity, squal-treshold
	if (mouse.data.delta_x == 0 && mouse.data.delta_y == 0 && mouse.data.squal < 25) {
		if (mouse.inactive < MOUSE_FILTER_LENGTH/2) {
			mouse.inactive += 1;
			if(mouse.inactive > MOUSE_FILTER_LENGTH/4){
				//mouse filter has no proper measurement
				mouse.proper_measurement=0;
			}
		}	
	} else {
		//fill history with new measurement until inactive reaches zero
		mouse.inactive += 1;
		int totalLag = mouse.inactive;
		while (mouse.inactive > 0) {
			//save value to history, but divided by the passed time to reduce lag-spikes as the mouse sensor gives the total displacement since last measurement after a short lag
			(mouse.history)[mouse.start_of_history] = mouse.data.delta_x/totalLag;
			(mouse.history)[(mouse.start_of_history)+1] = mouse.data.delta_y/totalLag;
	
			//mark new start of history
			mouse.start_of_history = ((mouse.start_of_history) + 2)%MOUSE_FILTER_LENGTH;
			
			mouse.inactive--;
		} 
		mouse.proper_measurement=1;
	}
}

//Sliding Window with variable length according to mouse_filter_size
void mouse_sw_filter() {
	int i;
	mouse.filt_delta_x = 0;
	mouse.filt_delta_y = 0;
	for (i=0;i<MOUSE_FILTER_LENGTH;i=i+2) {
		mouse.filt_delta_x += mouse.history[i] * 2./(1.*MOUSE_FILTER_LENGTH);
		mouse.filt_delta_y += mouse.history[i+1] * 2./(1.*MOUSE_FILTER_LENGTH);
	}
}

//TODO: copy to config file
float mouse_fir_filter_values[6] = {0.132994352274502,0.0631445404846248,0.0741311005137050,0.0830784231644473,0.0895821179091828,0.0929182100298337};
void mouse_fir_filter() {
	//6 field FIR filter for the mouse measurements
	//Calculate new value
	float filt_mouse[2] = {0,0};

	int i = 0;
	for (i=0; i<6; i++) {
		filt_mouse[0] += (mouse.history[(mouse.start_of_history+(i*2))%24]+mouse.history[(mouse.start_of_history+22-(i*2))%24])*mouse_fir_filter_values[i];
		filt_mouse[1] += (mouse.history[(mouse.start_of_history+(i*2)+1)%24]+mouse.history[(mouse.start_of_history+22-(i*2)+1)%24])*mouse_fir_filter_values[i];
	}	
	
	//Export filtered speed
	mouse.filt_delta_x = filt_mouse[0];
	mouse.filt_delta_y = filt_mouse[1];
}

float gyro_fir_filter_values[6] = {0.132994352274502,0.0631445404846248,0.0741311005137050,0.0830784231644473,0.0895821179091828,0.0929182100298337};
void mouse_gyro_fir_filter() {

	//Save value to history
	mouse.gyros_history_roll[mouse.start_of_gyros_history] = imuData.gyro[0];
	mouse.gyros_history_pitch[mouse.start_of_gyros_history] = imuData.gyro[1];

	//Mark new start of history
	mouse.start_of_gyros_history = (mouse.start_of_gyros_history + 1)%12;

	//Calculate new value
	float filt_gyro[2] = {0,0};

	int i = 0;
	for (i=0; i<6; i++) {
		filt_gyro[0] += (mouse.gyros_history_roll[(mouse.start_of_gyros_history+i)%12]+mouse.gyros_history_roll[(mouse.start_of_gyros_history+11-i)%12])*gyro_fir_filter_values[i];
		filt_gyro[1] += (mouse.gyros_history_pitch[(mouse.start_of_gyros_history+i+1)%12]+mouse.gyros_history_pitch[(mouse.start_of_gyros_history+11-i)%12])*gyro_fir_filter_values[i];
	}

	//Export filtered speed
	mouse.gyros_filt[0] = filt_gyro[0];
	mouse.gyros_filt[1] = filt_gyro[1];
}


//Uses the gyro to compensate for the pitch&roll tilt, output in mouse (mouse.filt_delta_x/y)
void mouse_gyro_compensate() {

	//No need to compensate anything when we can not measure the movement on the ground
	if (mouse.inactive > 0) {
		mouse.gyrofilt_delta_x = 0;
		mouse.gyrofilt_delta_y = 0;
		return;
	}

	//Calulcate the compensated movement according to a turning point which is "Offset" meters away from the camera:
	// RealDisplacement = MeasuredDisplacement*height+MeasuredRotation*(Offset+Height)
	mouse.gyrofilt_delta_x = (mouse.filt_delta_x/CAMERA_FACTOR_X*imuData.altitude)*100-mouse.gyros_filt[1]*(0.25+imuData.altitude);
	mouse.gyrofilt_delta_y = (mouse.filt_delta_y/CAMERA_FACTOR_Y*imuData.altitude)*100+mouse.gyros_filt[0]*(0.25+imuData.altitude);
}


unsigned char mouse_read_pixel()
{
	return mouse_read(0x0b);	
}	

void mouse_send_image(){
    char input = 0;
    char image[362];
	unsigned char tm = 0;
    image[0] = 0x7f;
    while(1){
        if (mouse.sensor_available) {
            if(pixel_counter >= 361)
            {
                mouse_write(0x0b, 0x00);
                e_send_uart2_char(image,362);
                pixel_counter = 0;
                LED_ORNG = 1 - LED_ORNG;
            }
            else{
                tm = mouse_read(0x0b);
                if((tm&0x80) == 0x80) {
                    pixel_counter++;
                    image[pixel_counter] = tm;
                }
            }
        } else {
            char message[] = "mouse sensor not available\n\r";
            e_send_uart2_char(message,sizeof(message));
            __delay32(_200MILLISEC);
            LED_ORNG = 1 - LED_ORNG;
        }

        input = 0;
        if (e_getchar_uart2(&input)) {
            switch (input) {
                case '0': mouse_light_set(0); break;
                case '1': mouse_light_set(10); break;
                case '2': mouse_light_set(20); break;
                case '4': mouse_light_set(40); break;
                case '5': mouse_light_set(50); break;
                case '6': mouse_light_set(60); break;
                case '7': mouse_light_set(70); break;
                case '8': mouse_light_set(80); break;
                case '9': mouse_light_set(90); break;
                case 'F': 
                case 'f': mouse_light_set(100); break;
                default: break;
            }
        }
    }

}


void mouse_light_set(unsigned short intensity)
{	
	mouse.light_intensity = intensity;
	mouse.light_dutycycle = 500 - mouse.light_intensity*5; //calculate the PWM timing 
}


void clock_delay_us(unsigned int us) 
{

	unsigned int mips = 40;

	__asm__ volatile("inc %[us], %[us]\n\t"
					 "lsr %[mips], #2, w1\n\t"
					 "0:\n\t" 
					 "dec %[us], %[us]\n\t"
					 "bra z, 2f\n\t"
					 "mov 2, WREG\n\t" /* /!\ 2 is here for w1 address. since the assembler does not accept a mov w1, WREG to read w1 by accessing the file register so the flags are modified I have hardcoded the address into the assembler mnemonic */
					 "1:\n\t"
					 "bra z, 0b\n\t"
					 "dec w0,w0\n\t"
					 "bra 1b\n\t"
					 "2:\n\t" 
					: /* No output*/ 
					:[us] "r" (us), [mips] "r" (mips)  /* Input */
					: "w0", "w1" /* We modify w0 and w1 */, "cc" /* alter flags */); 
}


/* TIMER INTERRUPT FOR THE MOUSE LIGHT*/
void __attribute__((interrupt, auto_psv)) _T2Interrupt(void)
{
	OC5RS = mouse.light_dutycycle; // Write Duty Cycle value for next PWM cycle
	IFS0bits.T2IF = 0; // Clear Timer 2 interrupt flag
}
