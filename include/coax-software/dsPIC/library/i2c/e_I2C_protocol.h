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

#ifndef _I2C_PROTOCOL
#define _I2C_PROTOCOL

#include "i2c/e_I2C_master_module.h"



void e_i2cp_init(void);
char e_i2cp_write (char device_add,char reg, char value);
char e_i2cp_read(char device_add,char reg);

//char e_i2cp_read_string(char device_add, unsigned char read_buffer[], char start_address, char string_length);
char e_i2cp_read_string(char device_add, char *read_buffer, char start_address, char string_length);


unsigned int MastergetsI2C1(unsigned int length, unsigned char * rdptr, unsigned int i2c1_data_wait);

char e_i2cp_write_string (char device_add, unsigned char write_buffer[], char start_address, char string_length);
void write_ext_eeprom(unsigned short address, char data);
char read_ext_eeprom(unsigned short address);
void e_i2cp_enable(void);
void e_i2cp_disable(void);

#endif
