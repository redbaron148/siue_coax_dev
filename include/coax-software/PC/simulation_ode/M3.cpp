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
#include "M3.h"

#define E(ii,jj) data[ii*3+jj]
#define F(m,ii,jj) m.data[ii*3+jj]

#ifdef WIN32
#include <windows.h>
#include <float.h>
#define isnan _isnan
#else
#endif

bool M3::check() const
{
	unsigned int i,j;
	for (i=0;i<3;i++) 
		for (j=0;j<3;j++)
			assert(!isnan(E(i,j)));
	return true;
}
	

M3::M3() 
{
	unsigned int i,j;
	for (i=0;i<3;i++) 
		for (j=0;j<3;j++)
			E(i,j) = 0;
	check();
}

M3::M3(const M3 & m3)
{
	unsigned int i,j;
	for (i=0;i<3;i++) 
		for (j=0;j<3;j++)
			E(i,j) = F(m3,i,j);
	check();
}

M3::M3(const V3 & v3)
{
	unsigned int i,j;
	for (i=0;i<3;i++) 
		for (j=0;j<3;j++)
			E(i,j) = (i==j)?v3(i):0;
	check();
}

M3::M3(const V3 & u,const V3 & ut)
{
	unsigned int i,j;
	for (i=0;i<3;i++) {
		for (j=0;j<3;j++) {
			E(i,j) = u(i)*ut(j);
		}
	}
	check();
}

M3::M3(double a00, double a01, double a02,
		double a10, double a11, double a12,
		double a20, double a21, double a22)
{
#define sm(i,j) E(i,j) = a##i##j
	sm(0,0); sm(0,1); sm(0,2);
	sm(1,0); sm(1,1); sm(1,2);
	sm(2,0); sm(2,1); sm(2,2);
#undef sm
	check();
}

M3::M3(const V3 & col0, const V3 & col1, const V3 & col2)
{
#define sm(i,j) E(i,j) = col##j(i)
	sm(0,0); sm(0,1); sm(0,2);
	sm(1,0); sm(1,1); sm(1,2);
	sm(2,0); sm(2,1); sm(2,2);
#undef sm
	check();
}

V3 M3::column(unsigned int i) const
{
	V3 out(E(0,i), E(1,i), E(2,i));
	return out;
}

double M3::operator()(unsigned int i, unsigned int j) const
{
	assert(i<3);
	assert(j<3);
	return E(i,j);
}

void M3::set(unsigned int i, unsigned int j, double val)
{
	assert(i<3);
	assert(j<3);
	E(i,j) = val;
	check();
}

const M3 & M3::operator=(const M3 & m3)
{
	unsigned int i;
	for (i=0;i<9;i++) data[i] = m3.data[i];
	check();
	return *this;
}

void M3::operator+=(const M3 & m3)
{
	unsigned int i,j;
	for (i=0;i<3;i++) 
		for (j=0;j<3;j++)
			E(i,j) += F(m3,i,j);
	check();
}

void M3::operator-=(const M3 & m3)
{
	unsigned int i,j;
	for (i=0;i<3;i++) 
		for (j=0;j<3;j++)
			E(i,j) -= F(m3,i,j);
	check();
}

void M3::operator*=(double s)
{
	unsigned int i,j;
	for (i=0;i<3;i++) 
		for (j=0;j<3;j++)
			E(i,j) *= s;
	check();
}

void M3::operator/=(double s)
{
	unsigned int i,j;
	for (i=0;i<3;i++) 
		for (j=0;j<3;j++)
			E(i,j) /= s;
	check();
}

void M3::operator*=(const M3 & m3)
{
	double newmb[9];
	unsigned int i,j,k;
	for (i=0;i<3;i++) {
		for (j=0;j<3;j++) {
			newmb[j*3+i] = 0;
			for (k=0;k<3;k++) {
				newmb[j*3+i] += F(m3,j,k) * E(k,i);
			}
		}
	}
	for (i=0;i<9;i++) {
		data[i] = newmb[i];
	}

	check();
}

M3::~M3()
{
}

void M3::zero()
{
	unsigned int i,j;
	for (i=0;i<3;i++) 
		for (j=0;j<3;j++)
			E(i,j) = 0;
	check();
}

void M3::identity()
{
	unsigned int i,j;
	for (i=0;i<3;i++) 
		for (j=0;j<3;j++)
			E(i,j) = (i==j)?1:0;
	check();
}

void M3::rotation(double x, double y, double z)
{
	/* z axis */
	identity();
	set(0,0, cos(z)); set(0,1,-sin(z));
	set(1,0, sin(z)); set(1,1, cos(z));
	M3 R; 
	/* y axis */
	R.identity();
	R.set(0,0, cos(y)); R.set(0,2, sin(y));
	R.set(2,0,-sin(y)); R.set(2,2, cos(y));
	(*this) *= R;
	/* x axis */
	R.identity();
	R.set(1,1, cos(x)); R.set(1,2,-sin(x));
	R.set(2,1, sin(x)); R.set(2,2, cos(x));
	(*this) *= R;
	check();
}

void M3::rodrigues(double x, double y, double z)
{
	// from : 
	// http://mathworld.wolfram.com/RodriguesRotationFormula.html
	V3 u(x,y,z);
	double theta = u.normL2(); 
	identity();
	if (u.normL2() != 0) {
		M3 om;
		u /= theta;
		om.set(0,0,   0);  om.set(0,1,-u(2)); om.set(0,2, u(1));
		om.set(1,0, u(2)); om.set(1,1,   0);  om.set(1,2,-u(0));
		om.set(2,0,-u(1)); om.set(2,1, u(0)); om.set(2,2,   0);
		M3 oms = om;
		oms *= sin(theta);
		operator+=(oms); // applied on this
		oms = om;
		oms *= om;
		oms *= (1-cos(theta));
		operator+=(oms); // applied on this
	}
	
	check();
}

void M3::transpose()
{
	double newmb[9];
	unsigned int i,j;
	for (i=0;i<3;i++) 
		for (j=0;j<3;j++)
			newmb[j*3+i] = E(j,i);
	for (i=0;i<9;i++) data[i] = newmb[i];
	check();
}

double M3::normL2() const
{
	unsigned int i,j;
	double S = 0;
	for (i=0;i<3;i++) {
		for (j=0;j<3;j++) {
			double v = E(i,j);
			S += v*v;
		}
	}
	return S;
}


void M3::print(FILE * fp) const
{
	unsigned int i,j;
	for (i=0;i<3;i++) {
		fprintf(fp,"[\t");
		for (j=0;j<3;j++) {
			fprintf(fp,"%+.3f\t",E(i,j));
		}
		fprintf(fp,"]\n");
	}
}


void M3::texprint(FILE * fp) const
{
	unsigned int i,j;
	fprintf(fp,"\\left(\\begin{array}{ccc}\n");
	for (i=0;i<3;i++) {
		for (j=0;j<2;j++) {
			fprintf(fp,"%+.3f & ",E(i,j));
		}
		fprintf(fp,"%+.3f \\\\ \n",E(i,j));
	}
	fprintf(fp,"\\end{array}\\right)\n");
}

V3 M3::operator*(const V3 & v3)
{
	V3 out(v3);
	out *= *this;
	return out;
}

