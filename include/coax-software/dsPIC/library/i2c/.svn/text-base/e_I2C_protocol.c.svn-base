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


#include "i2c/e_I2C_protocol.h"

#include <configs/coax_config.h>

//======================================================
//=== READ the value of an external I2C Module
//======================================================

char e_i2cp_read(char device_add,char reg)
{
	char error=0;								// Initialize to no error
	char value;									// Value that will be returned

	while(!error) {							// While they are still some errors...
		error=1;								// We initilize the error to 1
		error&=e_i2c_start();					// We intiate a start and...
		error&=e_i2c_write(device_add);    		// ... write the I2C MODULE ADDRESS we want to read
		error&=e_i2c_write(reg);     			// ... and the REGISTER ADDRESS we want to read
		error&=e_i2c_restart();					// ... In order to indicate to the slave that he can send the data, we iniate a restart
		error&=e_i2c_write(device_add+1);   	// ... and rewrite the MODULE ADDRESS with change of data direction ([bit 0]=1)
		error&=e_i2c_read(&value);    			// ... and read the data sent by the slave (the I2C Module)
		e_i2c_nack();							// ... only 1 byte is being read, so send nack
		e_i2c_stop();             				// ... the tranfer is then terminated by a stop
		if(error) {
			break;
		}
		e_i2c_reset();
	}
   	return value;
}

//========================================================
//=== READ the multiple registers of an external I2C Module
//========================================================

char e_i2cp_read_string(char device_add, char *read_buffer, char start_address, char string_length)

{
	char error=0;
	error=1;
	error&=e_i2c_start();
	error&=e_i2c_write(device_add);    		// Device address
	error&=e_i2c_write(start_address);		// address of first register to be read
	error&=e_i2c_restart();
	error&=e_i2c_write(device_add+1);    	// To change data direction ([bit 0]=1)

	while (string_length) {
		error&=e_i2c_read(read_buffer);		// read a single byte
		read_buffer++;
		string_length--;

		if(string_length == 0) {             /* If last char, generate NACK sequence */
			error&=e_i2c_nack();			// the last byte to be read, must send nack
		} else {                       /* For other chars,generate ACK sequence */
			error&=e_i2c_ack();
		}
		while(I2C1CONbits.ACKEN == 1);    /* Wait till ACK/NACK sequence is over */
	}
	e_i2c_stop();             				// End read cycle
	return error;
}


//======================================================
//=== WRITE the value of an external I2C Module
//======================================================

char e_i2cp_write (char device_add,char reg, char value)
{
	char error=0;								// Initialize to no error

	while(!error) {								// While they are still some errors...
		error=1;								// We initilize the error to 1
		error&=e_i2c_start();					// We intiate a start and...
		error&=e_i2c_write(device_add);    		// ... write the I2C MODULE ADDRESS we want to write
		error&=e_i2c_write(reg);     	  		// ... and the REGISTER ADDRESS we want to write
		error&=e_i2c_write(value);       		// ... and send the value we want to write
		error&=e_i2c_stop();             		// ... the tranfer is then terminated by a stop

		if(error) {
			break;
		}
		e_i2c_reset();
	}
	return error;
}


//======================================================
//=== READ an external I2C 24AA32 EEPROM from Microchip
//======================================================

char read_ext_eeprom(unsigned short address)
{
	char error=0;								// Initialize to no error
	char data=0x00;								// Initialize the data

	while(!error) { 							// While they are still some errors...
		error=1;								// We initilize the error to 1
		//while(!ext_eeprom_ready());			// Proposed in 2432.c from PICC, not very useful here
		error&=e_i2c_start();					// We intiate a start and...
		error&=e_i2c_write(0x10);				// ... write the hardware selectable address A0 = 0, A1 = 1, A2 = 0 and we write (last bit = 0)
		error&=e_i2c_write(0x01);	// ... then the least significant byte of the memory address 
		error&=e_i2c_restart();					// ... In order to indicate to the slave that he can send the data, we iniate a restart
		error&=e_i2c_write(0x11);				// ... write the hardware selectable address A0 = 0, A1 = 1, A2 = 0 and we read (last bit = 1)
		error&=e_i2c_read(&data);				// ... and read the data sent by the slave (here the memory)
		error&=e_i2c_nack();					// ... only 1 byte is being read, so send nack
		error&=e_i2c_stop();					// ... the tranfer is then terminated by a stop
	}
	return(data);
}


//======================================================
//=== READ an external I2C 24AA32 EEPROM from Microchip
//======================================================

//char read_ext_eeprom(unsigned short address)
//{
//	char error=0;								// Initialize to no error
//    char data=0x00;								// Initialize the data
//
//	while(!error)								// While they are still some errors...
//	{
//		error=1;								// We initilize the error to 1
//     	//while(!ext_eeprom_ready());			// Proposed in 2432.c from PICC, not very useful here
//     	error&=e_i2c_start();					// We intiate a start and...
//		error&=e_i2c_write(0xa4);				// ... write the hardware selectable address A0 = 0, A1 = 1, A2 = 0 and we write (last bit = 0)
//		error&=e_i2c_write(uint_msb(address));	// ... write the most significant byte of the memory address
//		error&=e_i2c_write(uint_lsb(address));	// ... then the least significant byte of the memory address 
//		error&=e_i2c_restart();					// ... In order to indicate to the slave that he can send the data, we iniate a restart
//		error&=e_i2c_write(0xa5);				// ... write the hardware selectable address A0 = 0, A1 = 1, A2 = 0 and we read (last bit = 1)
//		error&=e_i2c_read(&data);				// ... and read the data sent by the slave (here the memory)
//		error&=e_i2c_nack();					// ... only 1 byte is being read, so send nack
//		error&=e_i2c_stop();					// ... the tranfer is then terminated by a stop
//	}
//   return(data);
//}
//
//======================================================
//=== WRITE an external I2C 24AA32 EEPROM from Microchip
//======================================================

void write_ext_eeprom(unsigned short address, char data)
{
 	char error=0;								// Initialize to no error

	while(!error)								// While they are still some errors...
	{
		error=1;								// We initilize the error to 1
    	//while(!ext_eeprom_ready());			// Proposed in 2432.c from PICC, not very useful here
		error&=e_i2c_start();					// We intiate a start and...
		error&=e_i2c_write(0x10);				// ... write the hardware selectable address A0 = 0, A1 = 1, A2 = 0 and we write (last bit = 0)
		error&=e_i2c_write(0x00);	// ... write the most significant byte of the memory address
		error&=e_i2c_write(0x05);				// ... and finally the value that we want to store
		error&=e_i2c_stop();					// ... the tranfer is then terminated by a stop
	}
//	delay_ms(8);
}
////======================================================
////=== WRITE an external I2C 24AA32 EEPROM from Microchip
////======================================================
//
//void write_ext_eeprom(unsigned short address, char data)
//{
// 	char error=0;								// Initialize to no error
//
//	while(!error)								// While they are still some errors...
//	{
//		error=1;								// We initilize the error to 1
//    	//while(!ext_eeprom_ready());			// Proposed in 2432.c from PICC, not very useful here
//		error&=e_i2c_start();					// We intiate a start and...
//		error&=e_i2c_write(0xa4);				// ... write the hardware selectable address A0 = 0, A1 = 1, A2 = 0 and we write (last bit = 0)
//		error&=e_i2c_write(uint_msb(address));	// ... write the most significant byte of the memory address
//		error&=e_i2c_write(uint_lsb(address));	// ... then the least significant byte of the memory address 
//		error&=e_i2c_write(data);				// ... and finally the value that we want to store
//		error&=e_i2c_stop();					// ... the tranfer is then terminated by a stop
//	}
//	delay_ms(8);
//}

//======================================================
//=== Check if external EEPROM is ready for transfert
//======================================================

//char ext_eeprom_ready() {
//   int ack;
//   e_i2c_start();            					// If the write command is acknowledged,
//   ack = e_i2c_write(0xa4);  					// then the device is ready.
//   e_i2c_stop();
//   return !ack;
//}

//======================================================
//=== Functions to initiate, enable or disable the bus
//======================================================

void e_i2cp_init(void)
{
	e_i2c_init();
}
void e_i2cp_enable(void)
{
	e_i2c_enable();
}

void e_i2cp_disable(void)
{
	e_i2c_disable();
}
