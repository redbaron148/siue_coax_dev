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

#include <libpic30.h>
#include "spi.h"
#include <p33FJ256GP506.h>

#include "configs/coax_ports.h"
#include <configs/coax_config.h>



void init_SPI(void)
{

	IPC8bits.SPI2IP = 6;			// SPI interrupt priority
//	SPI2CON1 = 0x0129;	            // 
	SPI2CON1 = 0b0000000100101001;	// prescaler 16:1 , 2dary presclaer 6:1
	SPI2STATbits.SPIROV = 0;		// Clear Overflow bit
	SPI2STATbits.SPIEN = 1;			// Enable the peripheral and configures the spi lines
}


void ByteWriteSPI2(unsigned char Add, unsigned char Data )
{
	DS_CS_2 = 0;                             // Select Device
	WriteSPI2 ( Add );               // write address byte to EEPROM
	WriteSPI2 ( Data );                 // Write Byte to device
  	DS_CS_2 = 1;                             // Deselect device and initiate Write
	SPI2STATbits.SPITBF = 0;		// Transmit started, SPIxTXB is empty

	__delay32(_3MICROSEC); //we have to wait at least 1 SPI-Clock-Cycle before the next chip-select
					//@400kHz -> 2.5us, I wait 3us -> 120 wait-cycles)
}

void BurstWriteSPI2(unsigned char Address, unsigned char data0, unsigned char data1, 
                                                   unsigned char data2, unsigned char data3, 
                                                   unsigned char data4, unsigned char data5, 
                                                   unsigned char data6, unsigned char data7)
{
  DS_CS_2 = 0;                             // Select Device
  WriteSPI2 ( Address );               // write address byte to EEPROM
  WriteSPI2 ( data0 );                 // Write Byte to device
  WriteSPI2 ( data1 );                 // Write Byte to device
  WriteSPI2 ( data2 );                 // Write Byte to device
  WriteSPI2 ( data3 );                 // Write Byte to device
  WriteSPI2 ( data4 );                 // Write Byte to device
  WriteSPI2 ( data5 );                 // Write Byte to device
  WriteSPI2 ( data6 );                 // Write Byte to device
  WriteSPI2 ( data7 );                 // Write Byte to device
  DS_CS_2 = 1;                         // Deselect device and initiate Write
  SPI2STATbits.SPITBF = 0;

	__delay32(_3MICROSEC); //we have to wait at least 1 SPI-Clock-Cycle before the next chip-select
					//@400kHz -> 2.5us, I wait 3us -> 120 wait-cycles)

 }

void WriteSPI2(unsigned int data_out)
{  
   if (SPI2CON1bits.MODE16)          /* word write */
       SPI2BUF = data_out;
   else
       SPI2BUF = data_out & 0xff;    /*  byte write  */
   while(SPI2STATbits.SPITBF);
    while (!SPI2STATbits.SPIRBF); // Wait for the dummy byte clocked in during the command write to appear
   data_out = SPI2BUF;               //Avoiding overflow when reading
} 

void ByteReadSPI2(unsigned char Add, unsigned char *rdptr, unsigned char length )
{
  DS_CS_2 = 0;                             // Select Device
  WriteSPI2( Add );                // WRITE word address to EEPROM
  getsSPI2( rdptr, length );           // read in multiple bytes
  DS_CS_2 = 1;                             // Deselect Device

  __delay32(_3MICROSEC); //we have to wait at least 1 SPI-Clock-Cycle before the next chip-select
					//@400kHz -> 2.5us, I wait 3us -> 120 wait-cycles)
                             
}

void getsSPI2( unsigned char *rdptr, unsigned char length )
{
  while ( length )                  // stay in loop until length = 0
  {
    *rdptr++ = ReadSPI2();          // read a single byte
    length--;                       // reduce string length count by 1
  }
}

unsigned int ReadSPI2()
{         
  SPI2STATbits.SPIROV = 0;
  SPI2BUF = 0x00;                  // initiate bus cycle 
  while(!SPI2STATbits.SPIRBF);
   /* Check for Receive buffer full status bit of status register*/
  if (SPI2STATbits.SPIRBF)
  { 
      SPI2STATbits.SPIROV = 0;
              
      if (SPI2CON1bits.MODE16) {
          return (SPI2BUF);           /* return word read */
	  } else {
          return (SPI2BUF & 0xff);    /* return byte read */
	  }
  }
  return -1;                  		/* RBF bit is not set return error*/
}



