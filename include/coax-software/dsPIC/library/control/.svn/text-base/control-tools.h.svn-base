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
#ifndef CONTROL_TOOLS_H
#define CONTROL_TOOLS_H

typedef struct {
	float e,e_old;
	float u;
	float p;
	float i;
	float d;
	float kp;
	float ki;
	float kd;
} CONTROL_PID_VAR;

float CONTROL_filter(float data_new, float data_old, float T_filt, float tau_filter);
void CONTROL_pid_ctrl(CONTROL_PID_VAR *ctr_var, float T);
void CONTROL_pid_ctrl_alt(CONTROL_PID_VAR *ctr_var, float T);
void CONTROL_pid_ctrl_yaw(CONTROL_PID_VAR *ctr_var, float T);
void CONTROL_arwp(CONTROL_PID_VAR *ctr_var, float tresh_low, float tresh_high);
void CONTROL_arwp_sati(CONTROL_PID_VAR *ctr_var, float t_low, float t_high);
void CONTROL_ramp(CONTROL_PID_VAR *ctr_var, float ramp_step);

#endif // CONTROL_TOOLS_H
