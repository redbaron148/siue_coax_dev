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
#ifndef PID_CLASS_H
#define PID_CLASS_H

#include <math.h>
#include <string>

double now();

#ifdef WIN32
// implementation of remainder function. Defined in POSIX.C99
double remainder(double x, double y);
#endif

class PID
{
	protected:
		std::string name;
		double I;
		bool autodt;
		double dt, last_t;
		double Kp,Ki,Kd;
		double e, last_e;
		double Imax,Omax,Emax;
		bool angle;
	public:
		PID(const std::string & nm) : name(nm), I(0), autodt(true), dt(0), last_t(-1), 
			Kp(0), Ki(0), Kd(0), e(0), last_e(0),
			Imax(-1), Omax(-1), Emax(-1), angle(false) { }

		PID(const std::string & nm, double kp, double ki, double kd) 
			: name(nm), I(0), autodt(true), dt(0), last_t(-1), 
			Kp(kp), Ki(ki), Kd(kd), e(0), last_e(0),
			Imax(-1), Omax(-1), Emax(-1), angle(false) { }

		PID(const std::string & nm, double _dt, double kp, double ki, double kd) 
			: name(nm), I(0), autodt(false), dt(_dt), last_t(-1),
			Kp(kp), Ki(ki), Kd(kd), e(0), last_e(0),
			Imax(-1), Omax(-1), Emax(-1), angle(false) { }

		~PID() {}

		void setGains(double kp,double ki,double kd) {
			Kp=kp;
			Ki=ki;
			Kd=kd;
		}
		void setKp(double kp) {Kp=kp;}
		void setKi(double ki) {Ki=ki;}
		void setKd(double kd) {Kd=kd;}

		void setImax(double mx) {Imax=mx;}
		void setOmax(double mx) {Omax=mx;}
		void setEmax(double mx) {Emax=mx;}
		void reset() {I=0.0;}

		void setAngle(bool a) {angle=a;}

		double operator()(double ystar, double y);
		double operator()(double e);

		void print() const {
			printf("PID '%s': %.2g/%.2g/%.2g : I %.2f\n",
					name.c_str(),Kp,Ki,Kd,I);
		}

};


#endif // PID_CLASS_H
