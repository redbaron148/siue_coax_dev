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

#ifndef _I2C_MASTER_MODULE
#define _I2C_MASTER_MODULE

#include "p33FJ256GP506.h"

#define START			1
#define WRITE			2
#define ACKNOWLEDGE		3
#define READ			4
#define STOP			5
#define RESTART 		6
#define ERROR			10
#define OPERATION_OK	0

char e_i2c_init(void);
char e_i2c_start(void);
char e_i2c_restart(void);
char e_i2c_ack(void);
char e_i2c_nack(void);
char e_i2c_read(char *buf);
char e_i2c_stop(void);
char e_i2c_write(char byte);
char e_i2c_enable(void);
char e_i2c_disable(void);
char e_i2c_reset(void);

#endif
