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
* Description: This software interfaces the SPI module on the dsPIC33
*************************************************************/
#ifndef _SPI
#define _SPI

#include <p33FJ256GP506.h>

void init_SPI(void);
void WriteSPI2(unsigned int data_out) ;
void getsSPI2( unsigned char *, unsigned char );
unsigned int ReadSPI2( void );

void ByteWriteSPI2( unsigned char, unsigned char );
void BurstWriteSPI2( unsigned char,  unsigned char, unsigned char, unsigned char, unsigned char, unsigned char,  unsigned char,  unsigned char,  unsigned char);
void ByteReadSPI2( unsigned char, unsigned char * , unsigned char);



#endif
