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
* Description:This software is a control algorithm for the CoaX helicopter
*************************************************************/

#include "control-tools.h"

#define T_CTRL_delay_alt_pos 	2   	// times T_CTRL in [ms]			
#define T_CTRL_delay_alt_neg 	9   	// times T_CTRL in [ms]			
#define T_CTRL_delay_yaw 		0   	// times T_CTRL in [ms]
#define alt_i_th				0.0		// Threshold for the deadband of the altitude integrator [m]
static int T_ctrl_delay_cnt_pos = T_CTRL_delay_alt_pos;
static int T_ctrl_delay_cnt_neg = T_CTRL_delay_alt_neg;
// Functions ----------------------------------------------------

float CONTROL_filter(float data_new, float data_old, float T_filt, float tau_filter)
{ 
	float alpha; 		
	alpha = T_filt/(tau_filter+T_filt);
	data_new = alpha*data_new+(1-alpha)*data_old;
	return(data_new);	
}

void CONTROL_pid_ctrl(CONTROL_PID_VAR *ctr_var, float T)
{	
	ctr_var->p = ctr_var->kp * ctr_var->e;		 // P-Part

	ctr_var->i = ctr_var->i + ctr_var->e * ctr_var->ki;	 // I-Part

	ctr_var->d = ((ctr_var->e - ctr_var->e_old)/ T) * ctr_var->kd;

	ctr_var->u = ctr_var->p + ctr_var->i + ctr_var->d;							

	ctr_var->e_old = ctr_var->e;
}

void CONTROL_pid_ctrl_alt(CONTROL_PID_VAR *ctr_var, float T)
{	
	ctr_var->p = ctr_var->kp * ctr_var->e;		 // P-Part

	T_ctrl_delay_cnt_pos--;
	T_ctrl_delay_cnt_neg--;

	if (T_ctrl_delay_cnt_pos <= 0 && ctr_var->e > 0.0) // if climb
	{
		if (ctr_var->e > alt_i_th)		// if outside the band
		{
			ctr_var->i = ctr_var->i + ctr_var->e * ctr_var->ki;	 // I-Part
		}
		else
		{
			ctr_var->i = ctr_var->i;	 // I-Part
		}
		T_ctrl_delay_cnt_pos = T_CTRL_delay_alt_pos;
	}

	if (T_ctrl_delay_cnt_neg <= 0 && ctr_var->e < 0.0) // if descent
	{
		if (ctr_var->e < -alt_i_th)		// if outside the band
		{
			ctr_var->i = ctr_var->i + ctr_var->e * ctr_var->ki;	 // I-Part
		}
		else
		{
			ctr_var->i = ctr_var->i;	 // I-Part
		}
		T_ctrl_delay_cnt_neg = T_CTRL_delay_alt_neg;
	}

	ctr_var->d = ((ctr_var->e - ctr_var->e_old)/ T) * ctr_var->kd;

	ctr_var->u = ctr_var->p + ctr_var->i + ctr_var->d;					

	ctr_var->e_old = ctr_var->e;
}


void CONTROL_pid_ctrl_yaw(CONTROL_PID_VAR *ctr_var, float T)
{	
	// ctr_var->p_heading = ctr_var->kp_heading * ctr_var->e_heading;		 // P-Part

	ctr_var->i = ctr_var->i + ctr_var->e * ctr_var->ki;	 // I-Part

	ctr_var->d = ctr_var->kp * ctr_var->e;		 // D-Part

	// ctr_var->u = ctr_var->p_heading + ctr_var->i + ctr_var->d;							
	ctr_var->u = ctr_var->i + ctr_var->d;							

	ctr_var->e_old = ctr_var->e;
}


void CONTROL_arwp(CONTROL_PID_VAR *ctr_var, float tresh_low, float tresh_high)
{
	float t_low, t_high;

	t_low = tresh_low;
	t_high = tresh_high;

	if (ctr_var->u > t_high)
	{
		ctr_var->i = ctr_var->i + (t_high-ctr_var->u);
		ctr_var->u = t_high;
	}

	if (ctr_var->u < t_low)
	{
		ctr_var->i = ctr_var->i + (t_low-ctr_var->u);
		ctr_var->u = t_low;
	}
}

void CONTROL_arwp_sati(CONTROL_PID_VAR *ctr_var, float t_low, float t_high)
{
	if (ctr_var->i > t_high){
		ctr_var->i = t_high;
	}

	if (ctr_var->i < t_low){
		ctr_var->i = t_low;
	}
}

void CONTROL_ramp(CONTROL_PID_VAR *ctr_var, float ramp_step)
{
	if (ctr_var->e > ramp_step)		// ramp_step > 0
		ctr_var->e = ramp_step;
	else if (ctr_var->e < -ramp_step)
		ctr_var->e = -ramp_step;
}


