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

#ifndef VECTOR_3x3_H
#define VECTOR_3x3_H
// #include <gsl/gsl_vector.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

class V3;
class M3;

#include "M3.h"


class V3
{
	protected:
		double data[3];
	public:

		V3();
		V3(const V3 & v3);
		V3(double a, double b, double c);
		const V3 & operator=(const V3 & v3);
		double operator()(unsigned int i) const;
		void set(unsigned int i,double v);
		void operator+=(const V3 & v3);
		void operator-=(const V3 & v3);
		void operator*=(double s);
		void operator/=(double s);
		void operator*=(const M3 & m3);
		double operator*(const V3 & v3) const; /* dot product */

		V3 operator-() const;
		V3 operator+() const;
		V3 operator+(const V3 & v3) const;
		V3 operator-(const V3 & v3) const;
		V3 operator*(double s) const;
		V3 operator/(double s) const;
		V3 cross(const V3 & v3) const;
		V3 unit() const;

		double square() const; /* compute Vt * V */
		/* alias */
		double normL2() const {return sqrt(square());}

		~V3();

		void zero();

		void print(FILE * fp = stdout) const;
		void texprint(FILE * fp = stdout) const;
		bool check() const;
};

extern double angle(const V3 & u, const V3 & v);



#endif // VECTOR_3x3_H
