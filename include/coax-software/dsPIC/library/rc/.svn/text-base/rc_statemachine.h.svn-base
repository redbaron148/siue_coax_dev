/*************************************************************
*
* Low-level sensor and control library for the Skybotix AG
* helicopters, and particularly COAX
*
* Developed by:
*  * Samir Bouabdallah samir.bouabdallah@skybotix.ch
*  * Cedric Pradalier: cedric.pradalier@skybotix.ch
*  * Gilles Caprari: gilles.capraru@skybotix.ch
*  * Thomas Baumgartner
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
* Description:This software manages the RC receiver of the CoaX
*************************************************************/
#ifndef __coax_rc_statemachine_H_
#define __coax_rc_statemachine_H_

#include "coax_rc.h"

typedef enum {
    RC_INITIALIZING,
    RC_KILLED,
    RC_STOP,
    RC_AUTO,
    RC_MANUAL_IDLE,
    RC_MANUAL_FLY_READY,
    RC_MANUAL_FLY_RANGE,
    RC_MANUAL_FLY_THRUST,
    RC_MANUAL_FLY_COAXSPEED,
    RC_MANUAL_FLY_COUPLED
} RC_CONTROL_STATE;

#define RC_MANUAL_FLY_DEFAULT RC_MANUAL_FLY_RANGE

void RCInitStateMachine();
void RCSMSwitchToIdle(); // For emergency landing
RC_CONTROL_STATE RCSMGetState();
RC_CONTROL_STATE RCSMUpdateState(); // No arguments, the RC variable are read directly


#endif 
