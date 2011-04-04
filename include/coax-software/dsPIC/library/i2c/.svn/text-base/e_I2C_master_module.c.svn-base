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
* Description: This software allows using the I2C hardware module on a DsPic33
* in a master mode in a single master system.
*************************************************************/
#include <libpic30.h>
#include <p33FJ256GP506.h>
#include "i2c/e_I2C_master_module.h"
#include "utils/utils.h"
#include "configs/coax_config.h"

char e_i2c_mode;


void idle_i2c(void)
{
    /* Wait until I2C Bus is Inactive */
    while(I2C1CONbits.SEN || I2C1CONbits.PEN || I2C1CONbits.RCEN || I2C1CONbits.ACKEN || I2C1STATbits.TRSTAT);	
}

char e_i2c_init(void)
{
	I2C1CONbits.I2CEN = 0;			// Enables the I2C module and configures the SDA and SCL pins as serial port pins
	I2C1CONbits.I2CSIDL = 0;		// Continue module operation in idle mode
	I2C1CONbits.SCLREL = 1;			// Release the clock
	I2C1CONbits.IPMIEN = 0;			// Only acknowledge own address
	I2C1CONbits.A10M = 0;			// 7bit slave address
	I2C1CONbits.DISSLW = 1;			// 1=Slew rate control disabled (enable for 400kHz operation!)
	I2C1CONbits.SMEN = 0;			// Disable SMBus Input thresholds (set for 3.3V operation!)
	I2C1CONbits.GCEN = 0;			// Enable interrupt on a general address call
	I2C1CONbits.STREN = 0;			// Enable software or receive clock stretching (important when dealing with interrupts)
	I2C1CONbits.ACKDT = 0;			// Send ACK during acknowledge
	I2C1CONbits.ACKEN = 0;			// Acknowledge sequence not in progress
	I2C1CONbits.RCEN = 0;			// Receive sequence not in progress
	I2C1CONbits.PEN = 0;			// STOP condition not in progress
	I2C1CONbits.RSEN = 0;			// Repeated START condition not in progress
	I2C1CONbits.SEN = 0;			// START condition not in progress
    //I2C1BRG = 395;					// 100khz ((int)((20000000/FSCLI2CBUS)-1.0))
	I2C1BRG = 95;					// 400khz ((int)((20000000/FSCLI2CBUS)-1.0))
	IPC4bits.MI2C1IP = 5;			// set the master interrupt priority
	I2C1STATbits.I2COV = 0;      	// clear Overflow flag
	IEC1bits.SI2C1IE = 0;			// disables slave int
	I2C1CONbits.I2CEN = 1;			// Enables the I2C module and configures the SDA and SCL pins as serial port pins
	__delay32(_10MILLISEC);
	return 1;
}

char e_i2c_reset(void)
{
	I2C1CONbits.I2CEN=0;			// disable I2C and stop peripheric
	IFS1bits.MI2C1IF=0;				// 
	IFS1bits.SI2C1IF=0;				// 
	IEC1bits.SI2C1IE=0;				// 
	e_i2c_init();					// intit I2C
	e_i2c_enable();					//enable interrupt	
	__delay32(_100MICROSEC);
	return 1;
}

char e_i2c_enable(void)
{
	IFS1bits.MI2C1IF=0;			// clear master interrupt flag
	IEC1bits.MI2C1IE=1;			// enable master I2C interrupt
	__delay32(_20MICROSEC);		// original 20us
	return 1;
}

char e_i2c_disable(void)
{
	IFS1bits.MI2C1IF=0;			// clear master interrupt flag
	IEC1bits.MI2C1IE=0;			// disable master I2C interrupt
	return 1;
}

char e_i2c_start(void)
{
	long i;
	while(I2C2CONbits.SEN || I2C2CONbits.PEN || I2C2CONbits.RCEN || I2C2CONbits.ACKEN || I2C2CONbits.RSEN);
	e_i2c_mode=START;
	if(I2C1STATbits.P) {	
		I2C1CONbits.SEN=1;
		for(i=10000;i;i--) {
			if(!e_i2c_mode) {
				return 1;
			}
		}
		return 0;
	} else {
		return 0;
	}
}

char e_i2c_restart(void)
{
	long i;
	e_i2c_mode=RESTART;
	if(I2C1STATbits.S) {	
		I2C1CONbits.RSEN=1;
		for(i=10000;i;i--) {
			if(!e_i2c_mode) {
				return 1;
			}
		}
		return 0;
	} else {
		return 0;
	}
}

char e_i2c_ack(void)
{
//	long i;
	e_i2c_mode=ACKNOWLEDGE;

	// make sure I2C bus is inactive
    if(I2C1CONbits.SEN || I2C1CONbits.PEN || I2C1CONbits.RCEN || I2C1CONbits.ACKEN || I2C1CONbits.RSEN)	 {
		return 0;
	}

	// set ACK mode
	I2C1CONbits.ACKDT=0;
	I2C1CONbits.ACKEN=1;

	if(!e_i2c_mode)
		return 1;
	return 0;
}

char e_i2c_nack(void)
{
	long i;
	e_i2c_mode=ACKNOWLEDGE;

	// make sure I2C bus is inactive
    if(I2C1CONbits.SEN || I2C1CONbits.PEN || I2C1CONbits.RCEN || I2C1CONbits.ACKEN || I2C1CONbits.RSEN)	 {
		return 0;
	}

	// set NACK mode
	I2C1CONbits.ACKDT=1;
	I2C1CONbits.ACKEN=1;

	for(i=10000;i;i--) {
		if(!e_i2c_mode) {
			return 1;
		}
	}
	return 0;
}

char e_i2c_read(char *buf)
{
	long i=10000;
	char read_ok=0;
	e_i2c_mode=READ;

	for(i=10000;i;i--) {
		if(!(I2C1CONbits.SEN || I2C1CONbits.PEN || I2C1CONbits.RCEN || I2C1CONbits.ACKEN || I2C1STATbits.TRSTAT))
		{
			read_ok=1;
			break;
		}
	}
	if(!read_ok) {
		return 0;
	}
		
	// start receive mode for I2C	
	I2C1CONbits.RCEN=1;
	
	// keep polling for I2C interrupt
	for(i=100000;i;i--) {
		if(!e_i2c_mode)	{ // once I2C interrupt is tripped, read buffer and return 1
			*buf=I2C1RCV;
			return 1;
		}
	}
	return 0;
}

char e_i2c_stop(void)
{
	long i;
	e_i2c_mode=STOP;

	I2C1CONbits.PEN=1;

	for(i=10000;i;i--) {
		if(!e_i2c_mode) {
			return 1;
		}
	}
	return 0;
}


char e_i2c_write(char byte)
{
	long i;
	e_i2c_mode=WRITE;
	I2C1TRN=byte;
	I2C1CONbits.SEN=1;

	// poll for I2C interrupt
	for(i=10000;i;i--) {
		if(!e_i2c_mode)	{// return 1(transmisison OK) if interrupt was tripped)
			return 1;
		}
	}

	return 0;
}


// interrupt  routine: 
void _ISR_NOAUTOPSV _MI2C1Interrupt(void)
{
	IFS1bits.MI2C1IF=0;			// clear master interrupt flag
	e_i2c_mode=OPERATION_OK;	
}
