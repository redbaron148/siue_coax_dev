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
#include <string.h>

#include "utilities.h"

/* #define DEBUG */

SBApiSimpleContext * checkArgs(unsigned int out, signed int in, 
		int nlhs, mxArray * plhs[], int nrhs,const mxArray *prhs[])
{
	char tmp[1024];
	mxArray *field=NULL;
	PTR_CAST connection;
	if (nlhs != out) {
		sprintf(tmp,"This function has only %d output(s)\n",out);
		mexErrMsgTxt(tmp);
		return NULL;
	}
	if (in >= 0) {
		if (nrhs != in) {
			sprintf(tmp,"This function expect exactly %d input(s)\n",in);
			mexErrMsgTxt(tmp);
			return NULL;
		}
	} else {
		if (nrhs < -in) {
			sprintf(tmp,"This function expect at least %d input(s)\n",-in);
			mexErrMsgTxt(tmp);
			return NULL;
		}
	}
	if (nrhs == 0) {
		return NULL;
	}

	if (strcmp(mxGetClassName(prhs[0]),"SBConnection") != 0) {
		mexErrMsgTxt(tmp);
		return NULL;
	}
	/*  mexPrintf("Checked OK\n");  */
	/*   Converting the pointer  */
	field = mxGetField(prhs[0],0,"context");
	if (field == NULL) {
		mexErrMsgTxt("Class SBConnection does not have element 'id'");
		return NULL;
	}
	mxAssert(mxGetClassID(field)==PTR_CLASS,"Invalid Connection Handle");
	connection = *((PTR_CAST*)mxGetData(field));
	return (SBApiSimpleContext*)connection;
}

#define TESTCONTENT(c, f) (c & f)
static const char * MODE_FIELD[8]={
	"navigation", "communication", "oavoid", 
	"roll", "pitch", "yaw", "altitude", "horizontal"
};

static
mxArray * mxCreateScalarUChar(unsigned char value)
{
	mwSize dims[1] = {1};
	mxArray *tmp = mxCreateNumericArray(1,dims,mxUINT8_CLASS,mxREAL);
	*((unsigned char*)mxGetData(tmp)) = value;
	return tmp;
}


static
mxArray * mxCreateScalarUInt(unsigned long value)
{
	mwSize dims[1] = {1};
	mxArray *tmp = mxCreateNumericArray(1,dims,mxUINT32_CLASS,mxREAL);
	*((unsigned int*)mxGetData(tmp)) = value;
	return tmp;
}

static
mxArray * mxCreateIntVect(unsigned int n, const unsigned short *v)
{
	unsigned int i;
#if 0
	mwSize dims[1] = {n};
	mxArray *tmp = mxCreateNumericArray(1,dims,mxINT32_CLASS,mxREAL);
	long *p = ((long*)mxGetData(tmp));
#else
	mxArray *tmp = mxCreateDoubleMatrix(1,n,mxREAL);
	double * p = mxGetData(tmp);
#endif
	for (i=0;i<n;i++) p[i] = v[i];
	return tmp;
}

static
mxArray * mxCreateDoubleVect(unsigned int n, const float *v)
{
	unsigned int i;
	mxArray *tmp = mxCreateDoubleMatrix(1,n,mxREAL);
	double * p = mxGetData(tmp);
	for (i=0;i<n;i++) p[i] = v[i];
	return tmp;
}

mxArray * convertStateToMex(const SBHeliState * state)
{
	mxArray * tmp;
	mwSize dims[1] = {1};
	mxArray * result = mxCreateStructArray(1,dims,0,NULL);
	mxAddField(result,"content");
	mxSetField(result,0,"content",convertContentToMex(state->content));

	mxAddField(result,"errorFlags");
	mxSetField(result,0,"errorFlags", mxCreateScalarUChar(state->errorFlags));

	if (TESTCONTENT(state->content,SBS_MODES)) {
		tmp =  mxCreateStructArray(1,dims,8,MODE_FIELD);
		mxSetField(tmp,0,"navigation", mxCreateScalarUChar(state->mode.navigation));
		mxSetField(tmp,0,"communication", mxCreateScalarUChar(state->mode.communication));
		mxSetField(tmp,0,"oavoid", mxCreateScalarUChar(state->mode.oavoid));
		mxSetField(tmp,0,"roll", mxCreateScalarUChar(state->mode.rollAxis));
		mxSetField(tmp,0,"pitch", mxCreateScalarUChar(state->mode.pitchAxis));
		mxSetField(tmp,0,"yaw", mxCreateScalarUChar(state->mode.yawAxis));
		mxSetField(tmp,0,"altitude", mxCreateScalarUChar(state->mode.altAxis));
		mxAddField(result,"mode");
		mxSetField(result,0,"mode", tmp);
	}
	if (TESTCONTENT(state->content,SBS_TIMESTAMP)) {
		mxAddField(result,"timeStamp");
		mxSetField(result,0,"timeStamp", mxCreateScalarUInt(state->timeStamp));
	}
	if (TESTCONTENT(state->content,SBS_TIMEOUT)) {
		mxAddField(result,"controlTimeout");
		mxSetField(result,0,"controlTimeout", mxCreateScalarUInt(state->controlTimeout));
		mxAddField(result,"watchdogTimeout");
		mxSetField(result,0,"watchdogTimeout", mxCreateScalarUInt(state->watchdogTimeout));
	}
	if (TESTCONTENT(state->content,SBS_RPY)) {
		mxAddField(result,"roll");
		mxSetField(result,0,"roll", mxCreateDoubleScalar(state->roll));
		mxAddField(result,"pitch");
		mxSetField(result,0,"pitch", mxCreateDoubleScalar(state->pitch));
		mxAddField(result,"yaw");
		mxSetField(result,0,"yaw", mxCreateDoubleScalar(state->yaw));
	}
	if (TESTCONTENT(state->content,SBS_GYRO)) {
		mxAddField(result,"gyro");
		mxSetField(result,0,"gyro", mxCreateDoubleVect(3,state->gyro));
	}
	if (TESTCONTENT(state->content,SBS_ACCEL)) {
		mxAddField(result,"accel");
		mxSetField(result,0,"accel", mxCreateDoubleVect(3,state->accel));
	}
	if (TESTCONTENT(state->content,SBS_MAGNETO)) {
		mxAddField(result,"magneto");
		mxSetField(result,0,"magneto", mxCreateDoubleVect(3,state->magneto));
	}
	if (TESTCONTENT(state->content,SBS_IMUTEMP)) {
		mxAddField(result,"imutemp");
		mxSetField(result,0,"imutemp", mxCreateDoubleScalar(state->imutemp));
	}
	if (TESTCONTENT(state->content,SBS_ALTITUDE)) {
		mxAddField(result,"zrange");
		mxSetField(result,0,"zrange", mxCreateDoubleScalar(state->zrange));
		mxAddField(result,"zfiltered");
		mxSetField(result,0,"zfiltered", mxCreateDoubleScalar(state->zfiltered));
	}
	if (TESTCONTENT(state->content,SBS_PRESSURE)) {
		mxAddField(result,"pressure");
		mxSetField(result,0,"pressure", mxCreateDoubleScalar(state->pressure));
	}
	if (TESTCONTENT(state->content,SBS_HRANGES)) {
		mxAddField(result,"hranges");
		mxSetField(result,0,"hranges", mxCreateDoubleVect(4,state->hranges));
	}
	if (TESTCONTENT(state->content,SBS_XY_REL)) {
		mxAddField(result,"xrel");
		mxSetField(result,0,"xrel", mxCreateDoubleScalar(state->xrel));
		mxAddField(result,"yrel");
		mxSetField(result,0,"yrel", mxCreateDoubleScalar(state->yrel));
	}
	if (TESTCONTENT(state->content,SBS_BATTERY)) {
		mxAddField(result,"battery");
		mxSetField(result,0,"battery", mxCreateDoubleScalar(state->battery));
	}
	if (TESTCONTENT(state->content,SBS_O_ATTITUDE)) {
		mxAddField(result,"o_attitude");
		mxSetField(result,0,"o_attitude", mxCreateIntVect(3,state->o_attitude));
	}
	if (TESTCONTENT(state->content,SBS_O_ALTITUDE)) {
		mxAddField(result,"o_altitude");
		mxSetField(result,0,"o_altitude", mxCreateIntVect(1,&state->o_altitude));
	}
	if (TESTCONTENT(state->content,SBS_O_TOL)) {
		mxAddField(result,"o_tol");
		mxSetField(result,0,"o_tol", mxCreateIntVect(1,&state->o_tol));
	}
	if (TESTCONTENT(state->content,SBS_O_XY)) {
		mxAddField(result,"o_xy");
		mxSetField(result,0,"o_xy", mxCreateIntVect(2,state->o_xy));
	}
	if (TESTCONTENT(state->content,SBS_O_OAVOID)) {
		mxAddField(result,"o_oavoid");
		mxSetField(result,0,"o_oavoid", mxCreateIntVect(2,state->o_oavoid));
	}
	return result;
}

unsigned long extractContents(const mxArray * content)
{
	unsigned int i;
	unsigned long value = 0;
#ifdef DEBUG
	mexPrintf("extractContents: Entering with class id %s\n",
			mxGetClassName(content));
#endif
	if (mxIsCell(content)) {
		mwSize j,nd = mxGetNumberOfDimensions(content);
		unsigned int numel = nd?1:0;
		const mwSize *dims = mxGetDimensions(content);
#ifdef DEBUG
		mexPrintf("extractContents: OK, that's a CELL\n");
#endif
		for (j=0;j<nd;j++) {
			numel *= dims[j];
		}
#ifdef DEBUG
		mexPrintf("extractContents: %d dims, %d elements\n",nd, numel);
#endif
		/*   an array of string  */
		for (i=0;i<numel;i++) {
			char name[256];
			mxArray * Pi = mxGetCell(content, i);
			if (!mxIsChar(Pi)) {
				mexPrintf("extractContent: warning all cells must be string");
				continue;
			}
			mxGetString(Pi,name,255);
#ifdef DEBUG
			mexPrintf("Element %d: '%s'\n",i,name);
#endif

#ifdef LINUX
#define TEST_AND_ADD(m) mxGetString(Pi,name,255); \
	if (strcasecmp(name,#m)==0) {value = value | SBS_##m;}
#endif

#ifdef WIN32
#define TEST_AND_ADD(m) mxGetString(Pi,name,255); \
	if (stricmp(name,#m)==0) {value = value | SBS_##m;}
#endif


			TEST_AND_ADD(MODES);
			TEST_AND_ADD(TIMESTAMP);
			TEST_AND_ADD(RPY);
			TEST_AND_ADD(GYRO);
			TEST_AND_ADD(ACCEL);
			TEST_AND_ADD(MAGNETO);
			TEST_AND_ADD(IMUTEMP);
			TEST_AND_ADD(ALTITUDE);
			TEST_AND_ADD(PRESSURE);
			TEST_AND_ADD(HRANGES);
			TEST_AND_ADD(XY_REL);
			TEST_AND_ADD(BATTERY);
			TEST_AND_ADD(TIMEOUT);
			TEST_AND_ADD(O_ATTITUDE);
			TEST_AND_ADD(O_ALTITUDE);
			TEST_AND_ADD(O_TOL);
			TEST_AND_ADD(O_XY);
			TEST_AND_ADD(O_OAVOID);
		}
	} else {
		mexPrintf("extractContent: can only accept a cell of string");
	}
#ifdef DEBUG
	mexPrintf("extractContent: final value: %08X\n",value);
#endif
	return value;
}


mxArray* convertContentToMex(unsigned long content)
{
	unsigned long lc;
	unsigned int count = 0, idx;
	mxArray * mlist;
	
#ifdef DEBUG
	mexPrintf("convertContentToMex: content %ld\n",content);
#endif
	lc = content;
	while (lc) {
		if (lc & 1) count += 1;
		lc = lc >> 1;
	}
#ifdef DEBUG
	mexPrintf("convertContentToMex: count %d\n",count);
#endif

	mlist = mxCreateCellMatrix(count,1);
	if (mlist == NULL) {
		mexErrMsgTxt("Failed to allocate cell matrix for sensor list");
	}

#ifdef DEBUG
	mexPrintf("convertContentToMex: created cell matrix\n");
#endif
	
	lc = content; count = 0;idx = 0; 
	while (lc) {
		if (lc & 1) {
			mwIndex sub[2] = {count,0};
			mxSetCell(mlist, mxCalcSingleSubscript(mlist,2,sub),
					mxCreateString(sbContentString(1 << idx)));
			count += 1;
#ifdef DEBUG
			mexPrintf("convertContentToMex: added %d:'%s'\n",
					1<<idx, sbContentString(1 << idx));
#endif
		}
		lc = lc >> 1; 
		idx += 1;
	}
#ifdef DEBUG
	mexPrintf("convertContentToMex: filled cell matrix\n");
#endif

	return mlist;
}

