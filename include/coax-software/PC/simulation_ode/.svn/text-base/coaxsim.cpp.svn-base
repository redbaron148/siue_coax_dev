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


#include <vector>
#include <assert.h>

#ifdef WIN32
#include <windows.h>
#endif

#ifdef LINUX
#include <unistd.h>
#include <sys/time.h>
#endif

#include <time.h>

#include <ode/ode.h>
#include "drawstuff/drawstuff.h"

#include <com/sbchannel.h>
#include <com/sbstate.h>
#include <com/sbcommloop.h>

#include <vector>
#include <map>

#include "PID.h"
#include "V3.h"
#include "M3.h"

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#endif


// some constants
#define SWASH_PLATE_MAX (D2R(5))
#define SIDE (0.15f)	// side length of a box
#define MASS (0.3)	// mass of the coax
#define GRAVITY 9.81
#define R2D(X) ((X)*(180./M_PI))
#define D2R(X) ((X)*(M_PI/180.))
#define US_CONE (D2R(60)) // opening of the ultrasound sensor
#define HAS_RANGE_BACK


// dynamics and collision objects
static dSpaceID space,dispspace;
static dWorldID world;
static dGeomID bgeom;
static dBodyID body;
static dGeomID geom[6];
typedef std::map<unsigned int,dGeomID,std::less<unsigned int> > RayMap;
static RayMap ray;

static std::map<dGeomID,unsigned int,std::less<dGeomID> > invray;
typedef std::map<dGeomID,unsigned int,std::less<dGeomID> >::iterator rayiterator;
static dJointGroupID contactgroup;
static dGeomID ground;
static std::vector<dGeomID> decoration;
int printpose = 0;
int displayrays = 0;

static float voltage = 7.5;
static float drange = 0.1;
static float start_h = 0.20;
static float bbox[3] = {0.08,0.08,0.10};
static float mbox_r = 0.005;
static float mbox_h = 0.15;
static float rotor_r = 0.15;
static float rotor_h = 0.004;
static float shaft_r = 0.005;
static float shaft_h = 0.030;

// start simulation - set viewpoint

static void start()
{
	dAllocateODEDataForThread(dAllocateMaskAll);

	static float xyz[3] = {0.50,-0.50,0.50};
	static float hpr[3] = {135.0000f,-19.5000f,0.0000f};
	dsSetViewpoint (xyz,hpr);
	// printf ("Press 'e' to start/stop occasional error.\n");
}

double noisetaux=0,noisetauy=0;
double noisetaum1=0,noisetaum2=0, noisetauyaw=0;
double lastMessage[2] = {-1,-1};
unsigned int messagePeriod[2] = {0,0};
unsigned int light_intensity = 0;

double ray_measure[21];
double baseThrust = MASS*GRAVITY;
PID zpid("zpid",0.01,1.0,0.03,+0.8);
PID vzpid("vzpid",0.01,0.5,0.5,0);
PID yawpid("yawpid",0.01,3.0,1.0e-3,0);
PID vyawpid("vyawpid",0.01,3.0,1.0e-3,0);
PID vxpid("vxpid",0.03,0.2,0.02,-0.05);
PID vypid("vypid",0.03,0.2,0.02,-0.05);

struct MotorControl {
	double motor1;
	double motor2;
	double motors;
	double servo1;
	double servo2;
	double deltam;
	double vzstar;
}; 

MotorControl output = {0,0,0,0,0,0,0};
MotorControl input = {0,0,0,0,0,0,0};


SBHeliStateRaw heliState;
SBCommLoopSpec gcomm;
SBCommLoopSpec *comm = &gcomm;

// called when a key pressed

static void command (int cmd)
{
	printf("Key %d\n",cmd);
#if 1
	if (toupper(cmd) == 'Q') {
		input.servo1 += 1e-3;
	}
	if (toupper(cmd) == 'A') {
		input.servo1 = 0;
	}
	if (toupper(cmd) == 'Z') {
		input.servo1 -= 1e-3;
	}
	if (toupper(cmd) == 'W') {
		input.servo2 += 1e-3;
	}
	if (toupper(cmd) == 'S') {
		input.servo2 = 0;
	}
	if (toupper(cmd) == 'X') {
		input.servo2 -= 1e-3;
	}
	if (toupper(cmd) == 'E') {
		input.vzstar += 0.01;
	}
	if (toupper(cmd) == 'D') {
		input.vzstar = 0.0;
	}
	if (toupper(cmd) == 'C') {
		input.vzstar -= 0.01;
	}
	if (toupper(cmd) == 'T') {
		input.motors += 0.1;
	}
	if (toupper(cmd) == 'G') {
		input.motors = 0;
	}
	if (toupper(cmd) == 'B') {
		input.motors -= 0.1;
	}
	if (toupper(cmd) == 'R') {
		input.deltam += 0.1;
	}
	if (toupper(cmd) == 'F') {
		input.deltam = 0;
	}
	if (toupper(cmd) == 'V') {
		input.deltam -= 0.1;
	}
	if (toupper(cmd) == 'P') {
		printpose = !printpose;
	}
	if (toupper(cmd) == 'L') {
		displayrays = !displayrays;
	}
	if (cmd == '+') {
		gcomm.verbose += 1;
		printf("Verbosity = %d\n",gcomm.verbose);
	}
	if (cmd == '-') {
		gcomm.verbose -= 1;
		printf("Verbosity = %d\n",gcomm.verbose);
	}
#endif
}


// simulation loop

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
	int i,n;
	// Check if we're talking of one of the ray
	// printf("nearCallback: %p %p\n",o1,o2);
	rayiterator it1 = invray.find(o1);
	rayiterator it2 = invray.find(o2);
	if ((it1 != invray.end()) || it2 != invray.end()) {
		const int N = 10;
		rayiterator it = (it1 == invray.end())?it2:it1;
		// printf("There is a ray\n");
		if ((o1 != bgeom) && (o2 != bgeom)) {
			dContact contact[N];
			n = dCollide(o1,o2,N,&contact[0].geom,sizeof(dContact));
			// printf("%d contacts ",n);
			if (n == 1) {
				// printf("%f",contact[0].geom.depth);
				if ((ray_measure[it->second]<0) || 
						(contact[0].geom.depth < ray_measure[it->second])) {
					ray_measure[it->second] = contact[0].geom.depth;
				}
			}
		}
		// printf("\n");
		return;
	}

	// only collide things with the ground
	// if ((o1 != ground) && (o2 != ground)) {
	// 	return;
	// }
	// only collide things with the heli body. Ignore collision between other
	// object in the world
	if ((o1 != bgeom) && (o2 != bgeom)) {
		return;
	}

	const int N = 10;
	dContact contact[N];
	n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
	if (n > 0) {
		for (i=0; i<n; i++) {
			// contact[i].surface.mode = dContactSoftERP | dContactSoftCFM;
			contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
				dContactSoftERP | dContactSoftCFM | dContactApprox1;
			//contact[i].surface.mode = dContactSlip1 | dContactSlip2 |
			//	dContactApprox1 | dContactBounce;
			contact[i].surface.mu = 0.0; // dInfinity;
			contact[i].surface.bounce = 0.0;
			contact[i].surface.slip1 = 0.0;
			contact[i].surface.slip2 = 0.0;
			contact[i].surface.soft_erp = 5e-1;
			//contact[i].surface.soft_cfm = 9e-1;
			contact[i].surface.soft_cfm = 0.5;
			dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
			dJointAttach (c,
					dGeomGetBody(contact[i].geom.g1),
					dGeomGetBody(contact[i].geom.g2));
		}
	}
}

unsigned int minray(double *begin, double *end) {
	double mind = 1.5e+6;
	// printf("Min %p-%p: ",begin,end);
	while (begin != end) {
		if ((*begin>0) && (*begin < mind)) {
			mind = *begin;
		}
		// printf("%p: %.2f/%.2f ",begin,*begin,mind);
		begin ++;
	}
	// printf("-> %.2f\n",mind);
	if ((mind <= 0)||(mind > 1e+6)) {
		return 0xFFFF;
	} else {
		return (unsigned int)round(mind*1000.);
	}
}

void rotateRayBeam(unsigned int first, const V3 & pos, 
		const M3 & localframe,  double opening) 
{
	V3 u = localframe.column(0);
	dGeomRaySet(ray[first+0],pos(0),pos(1),pos(2), u(0),u(1),u(2));

	u = V3(cos(opening/2),sin(opening/2),0);
	u *= localframe;
	dGeomRaySet(ray[first+1],pos(0),pos(1),pos(2), 
			u(0), u(1), u(2));

	u = V3(cos(opening/2),-sin(opening/2),0);
	u *= localframe;
	dGeomRaySet(ray[first+2],pos(0),pos(1),pos(2), 
			u(0), u(1), u(2));

	u = V3(cos(opening/2),0,sin(opening/2));
	u *= localframe;
	dGeomRaySet(ray[first+3],pos(0),pos(1),pos(2), 
			u(0), u(1), u(2));

	u = V3(cos(opening/2),0,-sin(opening/2));
	u *= localframe;
	dGeomRaySet(ray[first+4],pos(0),pos(1),pos(2), 
			u(0), u(1), u(2));
}

static void simLoop (int pause)
{
	unsigned int i;
	dVector3 s;
	dReal rad,len;
	dsSetTexture (DS_WOOD);


	// Get system coordinates
	const dReal * pos1 = dBodyGetPosition(body);
	const dReal * rot1 = dBodyGetRotation(body);
	const dReal * rbody1 = dBodyGetAngularVel(body);
	const dReal * vbody1 = dBodyGetLinearVel(body);
	V3 vel(vbody1[0],vbody1[1],vbody1[2]);
	V3 P1(pos1[0],pos1[1],pos1[2]);
	if (printpose) {
		printf("Pose: %.2f %.2f %.2f\n",P1(0),P1(1),P1(2));
	}
	//
	// Adjust for geometry and soil flexibility
	P1.set(2, P1(2) - 0.12);
	V3 ux(rot1[0],rot1[4],rot1[8]);
	V3 uy(rot1[1],rot1[5],rot1[9]);
	V3 uz(rot1[2],rot1[6],rot1[10]);

	V3 rayP[5];
	rayP[0] = P1 - (uz * 0.0);
	// printf("rayP[0] %.2f %.2f %.2f\n",rayP[0](0),rayP[0](1),rayP[0](2));
	dGeomRaySet(ray[0],rayP[0](0),rayP[0](1),rayP[0](2), -uz(0),-uz(1),-uz(2));
	rayP[1] = P1 + (ux * drange);
	rotateRayBeam(1,rayP[1],M3(ux,uy,uz),US_CONE);
	rayP[2] = P1 - (uy * drange);
	rotateRayBeam(6,rayP[2],M3(-uy,ux,uz),US_CONE);
	rayP[3] = P1 + (uy * drange);
	rotateRayBeam(11,rayP[3],M3(uy,-ux,uz),US_CONE);
#ifdef HAS_RANGE_BACK
	rayP[4] = P1 - (ux * drange);
	rotateRayBeam(16,rayP[4],M3(-ux,-uy,uz),US_CONE);
#endif

	for (i=0;i<21;i++) {
		ray_measure[i] = -1;
	}
	dSpaceCollide(space,0,&nearCallback);
	dWorldQuickStep (world,0.01);
	//dWorldStep (world,0.01);




	M3 R(rot1[0],rot1[1],rot1[2],
			rot1[4],rot1[5],rot1[6],
			rot1[8],rot1[9],rot1[10]);
#if 0
	printf("Rotation: | %.2f %.2f %.2f | %.2f %.2f %.2f | %.2f %.2f %.2f |\n",
			rot1[0],rot1[1],rot1[2],
			rot1[4],rot1[5],rot1[6],
			rot1[8],rot1[9],rot1[10]);
#endif

	gcomm.timecount += 1;
	heliState.timeStamp = gcomm.timecount;


	double roll,rollstar,pitch,pitchstar,yaw,yawstar,z,zstar;
	double gyro[3],accel[3];

	heliState.rcChannel[0] = 0;
	heliState.rcChannel[1] = 1;
	heliState.rcChannel[2] = 2;
	heliState.rcChannel[3] = 3;
	heliState.rcChannel[4] = 10;
	heliState.rcChannel[5] = -10;
	heliState.rcChannel[6] = 1000;
	heliState.rcChannel[7] = -1000;

	yaw = -atan2(R(0,1),R(0,0)); heliState.yaw = round(yaw*1800./M_PI);
	yawstar = heliState.control.yaw*M_PI/1800.;

	pitch = asin(R(0,2)); heliState.pitch = round(pitch*1800./M_PI);
    pitchstar = 0;

	roll = atan2(R(2,1),R(2,2)); heliState.roll = round(roll*1800./M_PI);
    rollstar = 0;


	gyro[0] = rbody1[0];heliState.gyro[0]=gyro[0]*1800./M_PI;
	gyro[1] = rbody1[1];heliState.gyro[0]=gyro[1]*1800./M_PI;
	gyro[2] = rbody1[2];heliState.gyro[0]=gyro[2]*1800./M_PI;
	accel[0] = vbody1[0];heliState.accel[0]=accel[0]*1000;
	accel[1] = vbody1[1];heliState.accel[0]=accel[1]*1000;
	accel[2] = vbody1[2];heliState.accel[0]=accel[2]*1000;
	z = P1(2); zstar = heliState.control.altitude/1000.0;
	// if (z < 1e-2) exit(0);
	// printf("zstar: %f %d\n",zstar,heliState.control.altitude);

	heliState.zrange = (unsigned short)round((ray_measure[0]>=0)?(ray_measure[0]*1000.):0xFFFF);
	heliState.hranges[SB_RANGE_FRONT] = minray(ray_measure+1,ray_measure+6);
	heliState.hranges[SB_RANGE_RIGHT] = minray(ray_measure+6,ray_measure+11);
	heliState.hranges[SB_RANGE_LEFT] = minray(ray_measure+11,ray_measure+16);
#ifdef HAS_RANGE_BACK
	heliState.hranges[SB_RANGE_BACK] = minray(ray_measure+16,ray_measure+21);
#else
	heliState.hranges[SB_RANGE_BACK] = 0xFFFF;
#endif
#if 0
	printf("Ranges: %d %d %d %d %d %d\n",
			heliState.zrange,heliState.control.altitude,
			heliState.hranges[SB_RANGE_FRONT],
			heliState.hranges[SB_RANGE_RIGHT],
			heliState.hranges[SB_RANGE_LEFT],
			heliState.hranges[SB_RANGE_BACK]);
#endif
	heliState.zfiltered = z*1000.;
	heliState.pressure = 0.;
	heliState.battery = voltage*1000;
    // heliState.coaxspeed.state = 0; 
    heliState.coaxspeed.state = COAXSPEED_AVAILABLE;
    if (drand48() > 0.05) {
        heliState.coaxspeed.state |= COAXSPEED_VALID_MEASUREMENT;
    }
    heliState.coaxspeed.light = light_intensity;
    float vx=0,vy=0;
    vx = (vbody1[0]*cos(yaw) + vbody1[1]*sin(yaw));
    vy = (-vbody1[0]*sin(yaw) + vbody1[1]*cos(yaw));
    heliState.coaxspeed.vel_x = (int)(vx*1000);
    heliState.coaxspeed.vel_y = (int)(vy*1000);

	for (i=0;i<2;i++) {
		if ((messagePeriod[i]>0) && (now() - lastMessage[i] > messagePeriod[i]*1e-3)) {
			if (!gcomm.stateReady[i]) {
				// We only send if we finished sending the previous one
				if (sbStateEncode(&gcomm.serialisedState,0,&heliState)) {
					// printf("Failed to encode heli state\n");
					// TODO: Error management
				} else {
					gcomm.stateReady[i] = 1;
				}
				lastMessage[i] = now();
			}
		}
	}


	// Update state machine
	sbStateMachine(&gcomm,&heliState);

	// Compute helicopter control
	dReal velocity;
	if (heliState.mode.navigation == SB_NAV_RAW) {
		output.motor1 = heliState.setpoint.motor1;
		output.motor2 = heliState.setpoint.motor2;
		output.servo1 = heliState.setpoint.servo1;
		output.servo2 = heliState.setpoint.servo2;
	
	} else {
		// Reset the raw input, just in case
		heliState.setpoint.motor1 = heliState.setpoint.motor2 = 0;
		heliState.setpoint.servo1 = heliState.setpoint.servo2 = 0;

		// TODO: Implement control code here using set points:
		// Rstar, Pstar, Ystar, Hxstar, Hystar, Zstar and
		// Rctrl, Pctrl, Yctrl, Hctrl, Zctrl
		switch (heliState.mode.rollAxis) {
			case SB_CTRL_MANUAL:
				output.servo2 = input.servo2;
                vypid.reset();
				break;
			case SB_CTRL_POS:
                rollstar = -heliState.control.roll*M_PI/1800.;
				output.servo2 = rollstar;
                vypid.reset();
				break;
			case SB_CTRL_VEL:
                // Not correct, but will be OK for now
                if (heliState.coaxspeed.state == 
                        (COAXSPEED_AVAILABLE | COAXSPEED_VALID_MEASUREMENT)) {
                    rollstar = -heliState.control.roll/1000.;
                    output.servo2 = vypid(rollstar,vy);
                } else {
                    output.servo2 = 0;
                }
				break;
			case SB_CTRL_NONE:
			case SB_CTRL_REL:
			default:
				output.servo2 = 0;
                vypid.reset();
				break;
		}

		switch (heliState.mode.pitchAxis) {
			case SB_CTRL_MANUAL:
				output.servo1 = input.servo1;
                vxpid.reset();
				break;
			case SB_CTRL_POS:
                pitchstar = -heliState.control.pitch*M_PI/1800.;
				output.servo1 = pitchstar;
                vxpid.reset();
				break;
			case SB_CTRL_VEL:
                if (heliState.coaxspeed.state == 
                        (COAXSPEED_AVAILABLE | COAXSPEED_VALID_MEASUREMENT)) {
                    pitchstar = -heliState.control.pitch/1000.;
                    output.servo1 = vxpid(pitchstar,vx);
                } else {
                    output.servo1 = 0;
                }
				break;
			case SB_CTRL_NONE:
			case SB_CTRL_REL:
			default:
				output.servo1 = 0;
                vxpid.reset();
				break;
		}
        // printf("Vel %.3f %.3f Roll %.3f / %.3f %.3f Pitch %.3f / %.3f %.3f\n",
        //         vbody1[0],vbody1[1],rollstar,vy,output.servo2, 
        //         pitchstar,vx,output.servo1);

		switch (heliState.mode.yawAxis) {
			case SB_CTRL_MANUAL:
				output.deltam = input.deltam;
				break;
			case SB_CTRL_VEL:
				output.deltam = vyawpid(yawstar, gyro[2]);
				break;
			case SB_CTRL_POS:
				output.deltam = yawpid(yawstar, yaw);
				break;
			case SB_CTRL_REL:
			case SB_CTRL_NONE:
			default:
				output.deltam = 0;
                yawpid.reset();
                vyawpid.reset();
				break;
		}

		double motors;
		switch (heliState.mode.altAxis) {
			case SB_CTRL_MANUAL | SB_CTRL_POS:
			case SB_CTRL_MANUAL | SB_CTRL_REL:
			case SB_CTRL_MANUAL | SB_CTRL_VEL:
				motors = vzpid(input.vzstar,vbody1[2]);
                zpid.reset();
				break;
			case SB_CTRL_MANUAL | SB_CTRL_FORCE:
				motors = input.vzstar;
				break;
			case SB_CTRL_VEL:
				motors = vzpid(zstar,vbody1[2]);
                zpid.reset();
				break;
			case SB_CTRL_POS:
				velocity = zpid(zstar,z);
				motors = vzpid(velocity,vbody1[2]);
				break;
			case SB_CTRL_REL:
				velocity = zpid(zstar,ray_measure[0]);
				motors = vzpid(velocity,vbody1[2]);
				break;
			case SB_CTRL_NONE:
			default:
				motors = 0.0;
                zpid.reset();
                vzpid.reset();
				break;
		}
		heliState.o_altitude = (int)(motors * 1000.);

#if 0
		printf("Altitude: %.2f/%.2f RPY (deg) %.2f/%.2f %.2f/%.2f %.2f/%.2f\n",
				z,zstar,roll*180./M_PI,rollstar*180./M_PI,
				pitch*180./M_PI,pitchstar*180./M_PI,
				yaw*180./M_PI,yawstar*180./M_PI);
#endif
#if 0
		printf("Rays: %.2f %.2f %.2f %.2f %.2f %.2f -> %d\n",
				ray_measure[0], ray_measure[11], ray_measure[12],
				ray_measure[13], ray_measure[14], ray_measure[15], heliState.hranges[SB_RANGE_LEFT]);
#endif

#if 1
		// Compute the forces resulting from the rotors and gravity
		switch (heliState.mode.navigation) {
			case SB_NAV_STOP :
				output.motor1 = output.motor2 = 0;
				break;
			case SB_NAV_IDLE :
				output.motor1 = output.motor2 = baseThrust/4;
				break;
			case SB_NAV_RAW :
				// already processed above
				break;
			default:
				output.motor1 = (1+output.deltam+motors)*baseThrust/1;
				output.motor2 = (1-output.deltam+motors)*baseThrust/1;
				break;
		}
		// printf("Motors: %f %f\n",output.motor1,output.motor2);
#else
		output.motor1 = output.motor2 = (1+input.motors)*(baseThrust+motors)/2;
#endif
	}

	noisetaux += drand48()/100;
	noisetauy += drand48()/100;
	noisetaum1 += drand48()/90;
	noisetaum2 += drand48()/97;
	noisetauyaw += drand48()/110;


	// Compute principal axis
	output.servo1 = std::max(-SWASH_PLATE_MAX,std::min(SWASH_PLATE_MAX,output.servo1));
	output.servo2 = std::max(-SWASH_PLATE_MAX,std::min(SWASH_PLATE_MAX,output.servo2));
#if 0
	V3 a1(cos(output.servo1),0,sin(output.servo1));
	V3 a2(0,cos(output.servo2),sin(output.servo2));
	V3 vF = (a1.cross(a2)).unit();
#else
	double s1 = output.servo1 + sin(noisetaux)*1e-3;
	double s2 = output.servo2 + sin(noisetauy)*1e-3;
	V3 a1 = V3(1,0,s1).unit();
	V3 a2 = V3(0,1,s2).unit();
	V3 vF = V3(s1,s2,1).unit();
#endif
	/*
	printf("a1 %.2f %.2f %.2f a2 %.2f %.2f %.2f vF %.2f %.2f %.2f\n",
			a1(0),a1(1),a1(2),
			a2(0),a2(1),a2(2),
			vF(0),vF(1),vF(2));
	*/

	V3 axe_body = R * V3(0,0,1);
	V3 axe_rotor = R * vF;


	dReal force1 = output.motor1+sin(noisetaum1)*1e-1;
	dReal force2 = output.motor2+sin(noisetaum2)*1e-1;

	// printf("Z %.2f F1 %.2f F2 %.2f F %.2f\n",P1(2),force1,force2,force1+force2);

	if ((P1(2) < 1e-2) && (force1+force2<0)) {
		force1=force2=0.0;
	}

	V3 F1 = vF * force1;
	V3 F2 = vF * force2;
	V3 friction = vel * (-0.15);

	V3 F = F1 + F2;
	/*
	printf("F1 %.2f %.2f %.2f f %.2f %.2f %.2f F %.2f %.2f %.2f\n",
			F1(0),F1(1),F1(2),
			friction(0),friction(1),friction(2),
			F(0),F(1),F(2));
	*/
	
	dBodyAddRelForce(body,F(0),F(1),F(2));
	dBodyAddForce(body,friction(0),friction(1),friction(2));

	V3 CT = vF * ((force1-force2)*1e-3 + sin(noisetauyaw)*2e-4);
	dBodyAddRelTorque(body,CT(0),CT(1),CT(2));
	dBodyAddTorque(body,-1e-2*rbody1[0],-1e-2*rbody1[1],-1e-2*rbody1[2]);

	// Add a torque for vertical stability
	V3 Zabs(0,0,1);
	double angleFromVert = acos(uz * Zabs);
	V3 tdir = uz.cross(Zabs);
	dBodyAddTorque(body,2*angleFromVert*tdir(0),
			2*angleFromVert*tdir(1),2*angleFromVert*tdir(2));

	
	// simulate voltage drop. The higher the force, the faster the drop
	voltage *= (1-(1e-6+1e-5*(fabs(force1)+fabs(force2))));
 

	// Draw the helicopter
	if (displayrays) {
		RayMap::iterator it;
		dsSetColor(1,1,1);
		for (it=ray.begin();it!=ray.end();it++) {
			dVector3 pos,dir;
			float dspos[3], dsend[3];
			double len;
			dGeomRayGet(it->second,pos,dir);
			dspos[0] = pos[0]; dspos[1] = pos[1]; dspos[2] = pos[2];
			len = dGeomRayGetLength(it->second);
			dsend[0] = pos[0] + len*dir[0];
			dsend[1] = pos[1] + len*dir[1];
			dsend[2] = pos[2] + len*dir[2];
			dsDrawLine(dspos,dsend);
		}
	}
	dVector3 ptmp;
	dsSetColor (0.1,0.1,0.8);
	for (unsigned int i=0;i<decoration.size();i++) {
		dGeomBoxGetLengths(decoration[i],s);
		const dReal * pos = dGeomGetPosition(decoration[i]);
		const dReal * rot = dGeomGetRotation(decoration[i]);
		dsDrawBox (pos,rot,s);
	}

	dsSetColor (1,1,0);
	dGeomBoxGetLengths(geom[0],s);
	ptmp[0]=pos1[0]-(mbox_h/2)*axe_body(0);
	ptmp[1]=pos1[1]-(mbox_h/2)*axe_body(1);
	ptmp[2]=pos1[2]-(mbox_h/2)*axe_body(2);
	dsDrawBox (ptmp,dBodyGetRotation(body),s);
	dsSetColor (1,0.5,0);
	dGeomBoxGetLengths(geom[5],s);
	ptmp[0]+=(mbox_h/3)*ux(0);
	ptmp[1]+=(mbox_h/3)*ux(1);
	ptmp[2]+=(mbox_h/3)*ux(2);
	dsDrawBox (ptmp,dBodyGetRotation(body),s);

	dsSetColor (1,1,0);
	dGeomCylinderGetParams(geom[1],&rad,&len);
	ptmp[0]=pos1[0]-(bbox[2]/2)*axe_body(0);
	ptmp[1]=pos1[1]-(bbox[2]/2)*axe_body(1);
	ptmp[2]=pos1[2]-(bbox[2]/2)*axe_body(2);
	dsDrawCylinder (ptmp,dBodyGetRotation(body),len,rad);

	dReal pos2[3], rot2[12];
	pos2[0] = pos1[0] + /* (bbox[2]-mbox_h)/2*axe_body(0) +*/ (rotor_h+shaft_h/2)*axe_rotor(0);
	pos2[1] = pos1[1] + /* (bbox[2]-mbox_h)/2*axe_body(1) +*/ (rotor_h+shaft_h/2)*axe_rotor(1);
	pos2[2] = pos1[2] + /* (bbox[2]-mbox_h)/2*axe_body(2) +*/ (rotor_h+shaft_h/2)*axe_rotor(2);
	V3 a2n = (vF.cross(a1)).unit();
	rot2[0]=a1(0); rot2[1]=a2n(0); rot2[2]=vF(0); rot2[3]=0;
	rot2[4]=a1(1); rot2[5]=a2n(1); rot2[6]=vF(1); rot2[7]=0;
	rot2[8]=a1(2); rot2[9]=a2n(2); rot2[10]=vF(2); rot2[11]=0;

	dsSetColor (0,1,1);
	dGeomCylinderGetParams(geom[2],&rad,&len);
	ptmp[0]=pos2[0]-2*rotor_h*axe_rotor(0);
	ptmp[1]=pos2[1]-2*rotor_h*axe_rotor(1);
	ptmp[2]=pos2[2]-2*rotor_h*axe_rotor(2);
	dsDrawCylinder (ptmp,rot2,len,rad);
	dsSetColor (0,0,0);
	dGeomCylinderGetParams(geom[3],&rad,&len);
	dsDrawCylinder (pos2,rot2,len,rad);
	dsSetColor (0,1,1);
	dGeomCylinderGetParams(geom[4],&rad,&len);
	ptmp[0]=pos2[0]+2*rotor_h*axe_rotor(0);
	ptmp[1]=pos2[1]+2*rotor_h*axe_rotor(1);
	ptmp[2]=pos2[2]+2*rotor_h*axe_rotor(2);
	dsDrawCylinder (ptmp,rot2,len,rad);


	dJointGroupEmpty(contactgroup);
	usleep(10000);
	sbIncrementTime(&gcomm,&heliState,10);
}

void updateHeliState()
{
	// We don't do anything particular in the simulation
}

void setPeriodicState(unsigned int source, unsigned int period_ms)
{
	assert(source < 2);
	messagePeriod[source] = period_ms;
}

void *commthread(void*)
{
	sbCommLoop(&gcomm,&heliState);
	return NULL;
}

void errorOutput(unsigned int errorLevel, const char * message)
{
	switch (errorLevel) {
		case SB_COMMLOOP_CRITICAL:
			fprintf(stderr,"CRITICAL: %s\n",message);
			break;
		case SB_COMMLOOP_ERROR:
			fprintf(stderr,"ERROR: %s\n",message);
			break;
		case SB_COMMLOOP_WARNING:
			fprintf(stderr,"WARNING: %s\n",message);
			break;
		case SB_COMMLOOP_DEBUG2:
			if (gcomm.verbose>1) {
				DEBUG(message);
			}
			break;
		case SB_COMMLOOP_DEBUG:
			DEBUG(message);
			break;
		case SB_COMMLOOP_OK:
		default:
			break;
	}
}

// Get/Set control parameters, must return 0 on success, -1 if settings are
// ignored
int updateControlParams(int setParams, SBControlParametersMessage *params)
{
	printf("updateControlParams %d\n",setParams);
	if (setParams) {
		// ignored yaw Offset
		baseThrust = params->baseThrust;
		zpid.setGains(params->altitudeKp,params->altitudeKi,params->altitudeKd);
		vyawpid.setGains(params->yawKp,params->yawKi,params->yawKd);
	} else {
		params->yawOffset = 0;
		params->baseThrust = baseThrust*1000;
		params->altitudeKp = zpid.getKp()*1000;
		params->altitudeKi = zpid.getKi()*1000;
		params->altitudeKd = zpid.getKd()*1000;
		params->yawKp = vyawpid.getKp()*1000;
		params->yawKi = vyawpid.getKi()*1000;
		params->yawKd = vyawpid.getKd()*1000;
		sbCtrlParametersPrint(stdout,params);
	}
	return 0;
}

// Get/Set trim mode, must return 0 on success, -1 if settings are
// ignored
int updateTrimMode(int setMode, SBTrimModeMessage *mode)
{
	if (setMode) {
		return -1; // not implemented
	} else {
		mode->trimMode = SB_TRIM_FROM_RC;
		mode->rollTrim = 0;
		mode->pitchTrim = 0;
	}
	return 0;
}


void buildCapacities(unsigned short capacity[2])
{
	capacity[0] = 0;
	capacity[1] = 0;
 	sbAddContent(capacity,SBC_MODES, SBC_TIMESTAMP, SBC_TIMEOUT,
 			SBC_ALTITUDE, SBC_HRANGES, SBC_IMU_ALL, 
 			SBC_BATTERY, SBC_CHANNELS, SBC_COAXSPEED, SBC_ALL,
            SBC_END_OF_CONTENT);
    printf("Sensor capabilities: ");
    sbContent16bPrint(stdout,capacity);
    printf("\n");
}

void setLightSensor(unsigned short percent) {
    printf("Light sensor set at %d percent\n",percent);
    light_intensity = percent;
}

#define ITERS 40	// number of iterations //40 for STEPFAST, not used for QUICKSTEP!!!!
int main (int argc, char **argv)
{
	const double world_erp = 0.8; //also for the robot ?
	const double world_cfm = 1e-20;

	// s1pid.setAngle(true);
	// s2pid.setAngle(true);
	yawpid.setAngle(true);

	// setup pointers to drawstuff callback functions
	dsFunctions fn;
	fn.version = DS_VERSION;
	fn.start = &start;
	fn.step = &simLoop;
	fn.command = &command;
	fn.stop = 0;
	fn.path_to_textures = "textures";
	if(argc==2)
	{
		fn.path_to_textures = argv[1];
	}

	// create world
	dInitODE2(0);
	world = dWorldCreate();
	contactgroup = dJointGroupCreate(0);
	space = dSweepAndPruneSpaceCreate( 0, dSAP_AXES_XYZ );
	dispspace = dSweepAndPruneSpaceCreate( 0, dSAP_AXES_XYZ );
	dWorldSetGravity (world,0.0,0.0,-GRAVITY);
	dWorldSetCFM (world, world_cfm); 
	dWorldSetERP (world, world_erp); //Also sets the ERP of the suspension ???
	dWorldSetQuickStepNumIterations (world,ITERS);
	ground = dCreatePlane (space,0,0,1,0);

	dGeomID wall;
	wall = dCreateBox(space,0.1,200.0,1.0);
	dGeomSetPosition(wall,-1,0,0.5);
	decoration.push_back(wall);
	wall = dCreateBox(space,2.0,0.1,1.0);
	dGeomSetPosition(wall,0,1,0.5);
	decoration.push_back(wall);
	wall = dCreateBox(space,1.5,5.0,0.1);
	dMatrix3 R;
	R[0] = cos(D2R(20)); R[1] = 0; R[2] = -sin(D2R(20)); R[3] = 0;
	R[4] = 0; R[5] = 1; R[6] = 0; R[7] = 0;
	R[8] = sin(D2R(20)); R[9] = 0; R[10] = cos(D2R(20)); R[11] = 0;
	dGeomSetRotation(wall,R);
	dGeomSetPosition(wall,1,0,0.1);
	decoration.push_back(wall);

	zpid.setOmax(0.2);
	zpid.setImax(0.3);
	vzpid.setOmax(1.5);
	vzpid.setImax(3.0);

	dMass m;

	// The coax main body
	body = dBodyCreate (world);
	dMassSetBoxTotal (&m,MASS,bbox[0],bbox[1],bbox[2]);
	dBodySetMass (body,&m);
	dBodySetPosition (body,0,0,start_h+bbox[2]/2);
	//bgeom = dCreateBox(space,rotor_r,rotor_r,bbox[2]);
	//bgeom = dCreateCylinder(space,rotor_r,bbox[2]);
	bgeom = dCreateSphere(space,rotor_r);
	//bgeom = dCreateCylinder(space,rotor_r,bbox[2]+mbox_h+2*rotor_h+shaft_h);
	dGeomSetBody(bgeom,body);

	// The coax main body, created in display space in order to simplify
	// collision checking.
	geom[0] = dCreateBox(dispspace,bbox[0],bbox[1],bbox[2]);
	// The coax motor box
	geom[1] = dCreateCylinder(dispspace,mbox_r,mbox_h);
	// The coax rotor 1
	geom[2] = dCreateCylinder(dispspace,rotor_r,rotor_h);
	// The coax inter-rotor shaft
	geom[3] = dCreateCylinder(dispspace,shaft_r,shaft_h);
	// The coax rotor 2
	geom[4] = dCreateCylinder(dispspace,rotor_r,rotor_h);
	// The coax nose
	geom[5] = dCreateBox(dispspace,0.06,0.02,0.02);

	ray[0] = dCreateRay(space, 5.0);
	invray[ray[0]] = 0;
	for (unsigned int i=1;i<16;i++) {
		ray[i] = dCreateRay(space, 3.0);
		invray[ray[i]] = i;
	}
#ifdef HAS_RANGE_BACK
	for (unsigned int i=16;i<21;i++) {
		ray[i] = dCreateRay(space, 3.0);
		invray[ray[i]] = i;
	}
#endif

	sbCommLoopInit(&gcomm,0x0100);
	sbSetIMUVersion("ODE Simulator");
	buildCapacities(gcomm.capacities);
	// setup communication channels
	gcomm.setPeriodicState = setPeriodicState;
	gcomm.updateHeliState = updateHeliState;
	gcomm.updateControlParams = updateControlParams;
	gcomm.updateTrimMode = updateTrimMode;
    gcomm.setLight = setLightSensor;
	gcomm.landingStep_mm = 200;
	gcomm.error = errorOutput;
	// Channels are defined in the global variables
	sbChannelCreateSocketUDP(gcomm.channel+CHANNEL_0_ID,NULL,5123);
	sbChannelOpen(gcomm.channel+CHANNEL_0_ID);
#if 0
	sbChannelCreateSocketUDP(gcomm.channel+CHANNEL_1_ID,NULL,5124);
	sbChannelOpen(gcomm.channel+CHANNEL_1_ID);
    printf("Opened channel on UDP ports 5123 and 5124\n");
#else
	sbChannelCreateSerial(gcomm.channel+CHANNEL_1_ID,"/dev/ttyUSB1",115200,1);
	sbChannelOpen(gcomm.channel+CHANNEL_1_ID);
    printf("Opened channel on UDP ports 5123 and /dev/ttyUSB1 (115200)\n");
#endif

	pthread_t tid;
	pthread_create(&tid,NULL,commthread,NULL);

	// run simulation
	gcomm.verbose = 0;
	dsSimulationLoop (argc,argv,352,288,&fn);

	pthread_cancel(tid);

	sbChannelDestroy(gcomm.channel+CHANNEL_0_ID);
	sbChannelDestroy(gcomm.channel+CHANNEL_1_ID);

	dWorldDestroy (world);
	dCloseODE();
	return 0;
}


