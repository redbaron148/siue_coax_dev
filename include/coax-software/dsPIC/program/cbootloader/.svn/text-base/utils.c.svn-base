/**************************************
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
**************************************/

#include <p33FJ256GP506.h>
#include <libpic30.h>

#include "utils/utils.h"

#define MICROSEC   		40uL     		//  # of cycles for 1uSec delay (minumum is 11 cycles)
#define MILLISEC   		(1000*MICROSEC)	// # of cycles for 1mSec delay (minumum is 11 cycles)

/**************************************************************************/
/* FUNCTION:    Wait functions   	                                      */
/**************************************************************************/
void WaitMiliSec(unsigned int mils)		// TO REMOVE !!!
{
	__delay32(mils * MILLISEC);
}

void WaitMicroSec(unsigned int mics)
{
	__delay32(mics * MICROSEC);
}



