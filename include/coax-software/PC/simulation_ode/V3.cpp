/**************************************
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
**************************************/

#include <math.h>
#include <assert.h>
#include "V3.h"

#ifdef WIN32
#include <windows.h>
#include <float.h>
#ifndef __GNUC__
#define EPOCHFILETIME (116444736000000000i64)
#else
#define EPOCHFILETIME (116444736000000000LL)
#endif
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double remainder(double x,double y)
{
	double r = fmod(x,y);
	if (r >= y/2) r -= y;
	return r;
}

#define isnan _isnan
#else

#endif

double angle(const V3 & u, const V3 & v) 
{
	double dot = u * v;
	double nu = u.normL2();
	double nv = v.normL2();
	return remainder(acos(dot/(nu*nv)),2*M_PI);
}


bool V3::check() const
{
	unsigned int i;
	for (i=0;i<3;i++)  {
		assert(!isnan(data[i]));
	}
	return true;
}
	

V3::V3() 
{
	for (int i=0;i<3;i++) data[i] = 0;
	check();
}

V3::V3(const V3 & v3)
{
	for (int i=0;i<3;i++) data[i] = v3.data[i];
	check();
}


V3::V3(double a, double b, double c)
{
	data[0] = a;
	data[1] = b;
	data[2] = c;
	check();
}

void V3::set(unsigned int i, double val)
{
	assert(i<3);
	data[i] = val;
	check();
}

double V3::operator()(unsigned int i) const
{
	assert(i<3);
	return data[i];
}

const V3 & V3::operator=(const V3 & v3)
{
	for (int i=0;i<3;i++) data[i] = v3.data[i];
	check();
	return *this;
}

void V3::operator+=(const V3 & v3) 
{
	for (int i=0;i<3;i++) data[i] += v3.data[i];
	check();
}

void V3::operator-=(const V3 & v3) 
{
	for (int i=0;i<3;i++) data[i] -= v3.data[i];
	check();
}

void V3::operator*=(double s) 
{
	for (int i=0;i<3;i++) data[i] *= s;
	check();
}

void V3::operator/=(double s) 
{
	for (int i=0;i<3;i++) data[i] /= s;
	check();
}

void V3::operator*=(const M3 & m3)
{
	double newmb[3];
	for (int i=0;i<3;i++) {
		newmb[i] = 0;
		for (int j=0;j<3;j++) newmb[i] += m3(i,j) * data[j];
	}
	for (int i=0;i<3;i++) {
		data[i] = newmb[i];
	}
	check();
}

double V3::operator*(const V3 & v3) const
{
	double res;
	for (int i=0;i<3;i++) {
		res += data[i] * v3.data[i];
	}
	return res;
}

V3::~V3()
{
}

void V3::zero()
{
	for (int i=0;i<3;i++) data[i] = 0;
	check();
}

double V3::square() const
{
	double res=0;
	for (int i=0;i<3;i++) {
		res += data[i]*data[i];
	}
	return res;
}


void V3::print(FILE * fp) const
{
	unsigned int j;
	fprintf(fp,"[\t");
	for (j=0;j<3;j++) {
		fprintf(fp,"%+.2f\t",data[j]);
	}
	fprintf(fp,"]\n");
}

void V3::texprint(FILE * fp) const
{
	unsigned int j;
	fprintf(fp,"\\left(\\begin{array}{c}\n");
	for (j=0;j<3;j++) {
		fprintf(fp,"%+.2f\\\\",data[j]);
	}
	fprintf(fp,"\n\\end{array}\\right)\n");
}

V3 V3::operator-() const
{
	V3 out(-data[0],-data[1],-data[2]);
	return out;
}

V3 V3::operator+() const
{
	return *this;
}

V3 V3::operator+(const V3 & v3) const
{
	V3 out(*this);
	out += v3;
	return out;
}

V3 V3::operator-(const V3 & v3) const
{
	V3 out(*this);
	out -= v3;
	return out;
}

V3 V3::operator*(double s) const
{
	V3 out(*this);
	out *= s;
	return out;
}

V3 V3::operator/(double s) const
{
	V3 out(*this);
	out /= s;
	return out;
}

V3 V3::cross(const V3 & b) const
{
	const V3 & a = *this;
	V3 c(a(1)*b(2)-a(2)*b(1),
			a(2)*b(0)-a(0)*b(2),
			a(0)*b(1)-a(1)*b(0));
	return c;
}

V3 V3::unit() const
{
	V3 out(*this);
	out /= normL2();
	return out;
}



