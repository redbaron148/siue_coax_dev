/*************************************************************
*
* API and Communication library for the Skybotix AG
* helicopters, and particularly COAX
*
* Developed by Cedric Pradalier: cedric.pradalier@skybotix.ch
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
#ifndef SBMATLAB_UTILITIES_H
#define SBMATLAB_UTILITIES_H

#include "SBConnection.h"

#include <mex.h>

extern
SBApiSimpleContext * checkArgs(unsigned int out, signed int in, 
		int nlhs, mxArray * plhs[], int nrhs, const mxArray *prhs[]);

extern
mxArray * convertStateToMex(const SBHeliState * state);

extern
unsigned long extractContents(const mxArray * content);

extern
mxArray * convertContentToMex(unsigned long content);


#endif /*   SBMATLAB_UTILITIES_H  */
