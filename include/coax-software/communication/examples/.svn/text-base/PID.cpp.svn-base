/****************************************************
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
****************************************************/

#include <stdlib.h>
#include <stdio.h>

#include "PID.h"

#ifdef WIN32
#include <windows.h>
#ifndef __GNUC__
#define EPOCHFILETIME (116444736000000000i64)
#else
#define EPOCHFILETIME (116444736000000000LL)
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double remainder(double x,double y)
{
	double r = fmod(x,y);
	if (r >= y/2) r -= y;
	return r;
}

double now() {
    FILETIME        ft;
    LARGE_INTEGER   li;
	__int64         t;

	GetSystemTimeAsFileTime(&ft);
	li.LowPart  = ft.dwLowDateTime;
	li.HighPart = ft.dwHighDateTime;
	t  = li.QuadPart;       /* In 100-nanosecond intervals */
	t -= EPOCHFILETIME;     /* Offset to the Epoch time */
	return t * 1e-7;
}


#else
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
double now()
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec + 1e-6*tv.tv_usec;
}
#endif

//#define saturate_cmp(x,xmax) (((x)<-(xmax))?-(xmax):(((x)>(xmax))?(xmax):(x)))
//#define saturate(x,xmax) (((xmax)>0)?(saturate_cmp(x,xmax)):(x))

double saturate(double x, double xmax) {
	if (xmax > 0) {
		if (x > xmax) x = xmax;
		if (x < -xmax) x = -xmax;
	}
	return x;
}

double PID::operator()(double ystar, double y)
{
	e = ystar - y;
	if (angle) {
		e = remainder(e,2*M_PI);
	}
	return operator()(e);
}

double PID::operator()(double e)
{
	double output, de;

	saturate(e,Emax);
	if (autodt) {
		double t = now();
		if (last_t<0) {
			dt = 0;
		} else {
			dt = t - last_t;
		}
		last_t = t;
	}
	de = (e-last_e)/dt;
	last_e = e;
	I += e * dt;
	I = saturate(I,Imax);
	output = Kp*e + Ki*I + Kd*de;
	output = saturate(output,Omax);
	return output;
}

