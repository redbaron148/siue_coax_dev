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

#include "us/SRF10.h"
#include <p33FJ256GP506.h>
#include "i2c/e_I2C_protocol.h"
#include "utils/utils.h"


/******************************************************************************
* Function Name     :  
* Description       : 

* Parameters        :  
* Return Value      : 
******************************************************************************/
void init_srf10_ALT (void)
{
	e_i2cp_write(ADD_SRF10_ALT,REG_MG,max_gain);
	WaitMiliSec(2);
	e_i2cp_write(ADD_SRF10_ALT,REG_RR_MSB,max_range);
	WaitMiliSec(2);
}

/******************************************************************************
* Function Name     :  
* Description       : 

* Parameters        :  
* Return Value      : 
******************************************************************************/
void srf10_set_maxrange (char AdS, char rangereg)		// Range [mm]= (range reg x 43)+ 43
{
	e_i2cp_write(AdS,REG_RR_MSB,rangereg);
}

/******************************************************************************
* Function Name     :  
* Description       : 

* Parameters        :  
* Return Value      : 
******************************************************************************/
void srf10_req_range (char AdS, char uni)
{
	e_i2cp_write(AdS,REG_CMD,uni);
}

/******************************************************************************
* Function Name     :  
* Description       : 

* Parameters        :  
* Return Value      : 
******************************************************************************/
unsigned char srf10_get_range (char Ads)
{
unsigned char range;
	range=e_i2cp_read(Ads,REG_RR_LSB);	// only LSB is retrieved (max 254cm) (see later)
	if (range == 255) {	
		return (0);
	} else {
		return (range);
	}

}

/******************************************************************************
* Function Name     :  
* Description       : 

* Parameters        :  
* Return Value      : 
******************************************************************************/
void srf10_req_altitude (void)
{
	e_i2cp_write (ADD_SRF10_ALT,REG_CMD,CMD_RESULT_CM);
}

/******************************************************************************
* Function Name     :  
* Description       : 

* Parameters        :  
* Return Value      : 
******************************************************************************/
unsigned char srf10_get_altitude (void)
{
unsigned char range;
	range=e_i2cp_read(ADD_SRF10_ALT,REG_RR_LSB);	// only LSB is retrieved (max 254cm) (see later)
	if (range >= 255) {	
		return (0);			// Return 1 if the SRF10 is not responding (ranging for instance)
	} else {
		return (range);
	}

}
