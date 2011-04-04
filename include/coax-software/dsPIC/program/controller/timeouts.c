/*************************************************************
*
* Low-level controller with API server for the Skybotix AG
* helicopters, and particularly COAX
*
* Developed by:
*  * Cedric Pradalier: cedric.pradalier@skybotix.ch
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


#include <com/sbconst.h>
#include <com/sbmessage.h>
#include <com/sbstate.h>
#include <com/sbchannel.h>
#include <com/sbcommloop.h>


#include "globals.h"
#include "state.h"
#include "timeouts.h"

/********************* Time counting and timeout management *************************/
//
//

unsigned short waitcount = 0;

void wait(unsigned int length_ms) {
	waitcount = 0;
	while (waitcount < length_ms);
}


void timecount_increment() {
	waitcount += 1;
}

