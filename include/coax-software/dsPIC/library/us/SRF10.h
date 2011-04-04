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
* Description: This software interfaces the SRF10 sonar on the CoaX
*************************************************************/

#ifndef _SRF10
#define _SRF10

// Settings
#define max_gain	0x08		// gain to send to oa us
#define max_range	0x46		// binary for max range register = ((Range Register x 43mm) + 43mm)

#define ADD_SRF10_ALT			0xE0	// The base adress of: SRF10 Altitude (0x70)
//#define ADD_SRF10_OA1			0xE1	// The base adress of: SRF10 OA 1				
//#define ADD_SRF10_OA2			0xE2	// The base adress of: SRF10 OA 2				
//#define ADD_SRF10_OA3			0xE3	// The base adress of: SRF10 OA 3				
//#define ADD_SRF10_OA4			0xE4	// The base adress of: SRF10 OA 4				


#define REG_CMD					0x00	// Adress of: Command Reg
#define REG_MG					0x01	// Adress of: Max Gain Reg (dflt:16)
#define REG_RR_MSB				0x02	// Adress of: Range Reg MSB
#define REG_RR_LSB				0x03	// Adress of: Range Reg LSB


#define CMD_RESULT_IN     		0x50	// Result in inches
#define CMD_RESULT_CM     		0x51	// Result in centimeters
#define CMD_RESULT_MS     		0x52	// Result in microseconds


void init_srf10_ALT (void);
void srf10_set_maxrange (char, char);
void srf10_req_range (char, char);
unsigned char srf10_get_range (char);
void srf10_req_altitude (void);
unsigned char srf10_get_altitude (void);

#endif
