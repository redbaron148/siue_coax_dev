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

#ifndef MATRIX_3x3_H
#define MATRIX_3x3_H

#include <stdlib.h>
#include <stdio.h>
class M3;
class V3;

#include "V3.h"


class M3
{
	protected:
		double data[9];
	public:

		M3();
		M3(double a00, double a01, double a02,
				double a10, double a11, double a12,
				double a20, double a21, double a22);
		M3(const M3 & m3);
		M3(const V3 & v3); /* diagonal */
		M3(const V3 & col1, const V3 & col2, const V3 & col3); 
		M3(const V3 & u,const V3 & ut); /* u * ut */
		const M3 & operator=(const M3 & m3);
		void operator+=(const M3 & m3);
		void operator-=(const M3 & m3);
		void operator*=(double s);
		void operator/=(double s);
		/* Multiplication from the left: m <- m3 * m */
		void operator*=(const M3 & m3); 

		M3 operator +(const M3 & m3) const {
			M3 out(*this); out += m3; return out;
		}
		M3 operator -(const M3 & m3) const {
			M3 out(*this); out -= m3; return out;
		}
		M3 operator *(const M3 & m3) const {
			M3 out(m3); out *= *this; return out;
		}
		
		V3 operator*(const V3 & v3); 

		double operator()(unsigned int i, unsigned int j) const;
		void set(unsigned int i, unsigned int j, double val);
		V3 column(unsigned int i) const;

		~M3();

		void zero();
		void identity();
		void rotation(double x,double y, double z);
		void rodrigues(double x,double y, double z);

		void transpose();
		double normL2() const;


		void print(FILE * fp = stdout) const;
		void texprint(FILE * fp = stdout) const;
		bool check() const;

};



#endif // MATRIX_3x3_H
