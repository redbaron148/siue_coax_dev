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

#include "mex.h"
#include "SBConnection.h"
#include "utilities.h"



/* mexFunction is the gateway routine for the MEX-file. */ 
void
mexFunction( int nlhs, mxArray *plhs[],
             int nrhs, const mxArray *prhs[] )
{
	SBApiSimpleContext* connection = checkArgs(1,1,nlhs,plhs,nrhs,prhs);
	unsigned long list = 0;
	int res;
	
	sbLockCommunication(&connection->control);
	res = sbGetSensorList(&connection->control, &list);
	sbUnlockCommunication(&connection->control);
	if (res) {
		mexErrMsgTxt("Error while asking form sensor list");
	}
	mexPrintf("sbGetSensorList returned %d\n",res);
	
	plhs[0] = convertContentToMex(list);
}




