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
* Description: This software interfaces the MAxbotix analog sonar on the CoaX
*************************************************************/

#ifndef _SONAR_H
#define _SONAR_H


#define FRONT	12
#define LEFT	11
#define RIGHT	10

#define MIN_RANGE 0.0
#define MAX_RANGE 6.0

#define CALIB_SLOPE 0.0043
#define CALIB_SHIFT 0.2213

//=============================================================
// DECLARATIONS
//=============================================================

typedef struct
{
	unsigned int front;
	unsigned int left;
	unsigned int right;
} SONAR_INT_STRUCT;

typedef struct
{
	float front;
	float left;
	float right;
} SONAR_STRUCT;


extern SONAR_INT_STRUCT sonar_int;
extern SONAR_STRUCT sonar;

//=============================================================
// FUNCTION PROTOTYPES
//=============================================================
void updateSONAR_FRONT(void);
void updateSONAR_LEFT(void);
void updateSONAR_RIGHT(void);





#endif

