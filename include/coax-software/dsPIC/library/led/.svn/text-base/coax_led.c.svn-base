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
* Description:This software manages the LEDs of the CoaX
*************************************************************/

#include "configs/coax_ports.h"
#include "configs/coax_config.h"

#include <libpic30.h>
#include <p33FJ256GP506.h>


void FlashORNG (void)
{
	LED_ORNG=1;
	__delay32(_25MILLISEC);
	LED_ORNG=0;
	__delay32(_25MILLISEC);
}


void FlashRED (void)
{
	LED_RED=1;
	__delay32(_25MILLISEC);
	LED_RED=0;
	__delay32(_25MILLISEC);
}
