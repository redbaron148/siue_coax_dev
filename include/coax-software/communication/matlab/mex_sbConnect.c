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

#include "SBConnection.h"
#include <com/sbsimple.h>


static const char * CONN_FIELD[1]={"context"};


/* mexFunction is the gateway routine for the MEX-file. */ 
void
mexFunction( int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[] )
{
	int res = -999;
	char destination[256] = "/dev/rfcomm0";
	unsigned int port = 5123;
	unsigned int baudrate = 115200;
	SBApiSimpleContext * simple = NULL;
	mxArray *result=NULL, *field=NULL;

	if (nlhs != 1) {
		mexErrMsgTxt("sbConnect can only create a single object");
	}
	if (nrhs < 1) {
		mexErrMsgTxt("Class creator needs two arguments: <port name> and optionnally <baudrate>");
	}
	mxGetString(prhs[0],destination,255);

	simple = (SBApiSimpleContext*)calloc(1,sizeof(SBApiSimpleContext));
	if (!simple) {
		mexErrMsgTxt("Failed to allocate memory for SBApiSimpleContext");
	}
	sbSimpleDefaultContext(simple);

	strncpy(simple->device,destination,255);
	if (simple->device[0] == '/') {
		simple->commType = SB_COMMTYPE_SERIAL;
	} else {
		simple->commType = SB_COMMTYPE_UDP;
		if (nrhs > 1) {
			simple->commPort = (unsigned int)(mxGetScalar(prhs[1]));
		}
	}
	simple->initNavState = SB_NAV_STOP;
	simple->cmdTimeout = 30000;
	simple->ctrlTimeout = 10000;
	simple->altCtrlMode = SB_CTRL_REL;
	simple->yawCtrlMode = SB_CTRL_VEL;
#if 0
	simple->rollCtrlMode = SB_CTRL_MANUAL;
	simple->pitchCtrlMode = SB_CTRL_MANUAL;
#else
	simple->rollCtrlMode = SB_CTRL_POS;
	simple->pitchCtrlMode = SB_CTRL_POS;
#endif
	simple->oaMode = SB_OA_NONE;

	res = sbSimpleInitialise(simple);
	if (res) {
		mexErrMsgTxt("Failed to initialise connection");
	}
	
	result = mxCreateStructMatrix(1,1,1,CONN_FIELD);
	field = mxCreateNumericMatrix(1,1, PTR_CLASS,mxREAL);
	*((PTR_CAST*)mxGetData(field)) = (PTR_CAST)simple;
	mxSetField(result,0,"context",field);

	plhs[0] = result;
}


