#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

//IO stuff
#include <sys/ioctl.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef WIN32
#define sscanf sscanf_s
#define strcpy strcpy_s
#define strncpy strncpy_s
#endif

#include <string>
#include <vector>

#include <com/sbapi.h>
#include <com/sbsimple.h>

#define D2R(X) ((X)*M_PI/180.0)
#define R2D(X) ((X)*180.0/M_PI)
#define MAX(X,Y) (((X)<(Y))?(Y):(X))
#define MIN(X,Y) (((X)<(Y))?(X):(Y))
#define SQR(X) ((X)*(X))

//#define DEBUG(c) printf("Executing "#c"\n")
//#define DEBUG(c) res=0;printf("Executing "#c"\n");c;printf("Result %d\n",res)
#define DEBUG(c) res=0;c;if (res) printf("Result of "#c": %d\n",res)
#define CRITICAL(c) res=0;c;if (res) {printf("Result of "#c": %d\n",res); return res;}

#define THROTTLE_TRIM_LIMIT 0.10 // flying
#define CONTROL_NOISE 0.2

struct LogPacket
{
	unsigned long timeStamp;
	/** Current helicopter attitude */
	float roll, pitch, yaw;
	/** GYRO data */
	float gyro[3];
	/** Accelerometer data */
	float accel[3];
	/** Range measurement in the vertical direction */
	float zrange;
	/** Output of attitude control (semantic unclear) */
	float o_attitude[3];
	/** Output of altitude control, i.e. thrust to keep the helicopter affloat  */
	float o_altitude;
	/** battery voltage */
	float battery;

	/** ranges **/
	float ranges[4];

	float rcChannel[2];

	LogPacket() {}

	LogPacket(const SBHeliState & state) {
		timeStamp = state.timeStamp;
		roll = state.roll;
		pitch = state.pitch;
		yaw = state.yaw;
		zrange = state.zrange;
		o_altitude = state.o_altitude;
		battery = state.battery;
		for (unsigned int i=0;i<3;i++) {
			gyro[i] = state.gyro[i];
			accel[i] = state.accel[i];
			o_attitude[i] = state.o_attitude[i];
		}
		for (unsigned int i=0;i<4;i++) {
			ranges[i] = state.hranges[i];
		}
		rcChannel[0] = state.rcChannel[SB_RC_ROLL];
		rcChannel[1] = state.rcChannel[SB_RC_PITCH];
	}

	void serialize(FILE * fp) {
		/*            1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 15 17 18 19 20 21 22  */
		/*           ts  r  p  y g0 g1 g2 a0 a1 a2  z o1 o2 o3 oa  V r1 r2 r3 r4 rl pi  */
		fprintf(fp,"%lu %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e %e ",
				timeStamp,roll,pitch,yaw,gyro[0],gyro[1],gyro[2],
				accel[0],accel[1],accel[2],zrange,
				o_attitude[0],o_attitude[1],o_attitude[2],o_altitude,battery,
				ranges[0],ranges[1],ranges[2],ranges[3],rcChannel[0],rcChannel[1]);
		fprintf(fp,"\n");
	}

};
	

class SBController
{
	protected:
		SBApiSimpleContext *simple;
		unsigned long sensorList;
		int res;
		double desRoll,desPitch;
		std::vector<LogPacket> logbuffer;
		enum {
			HELI_ON_GROUND,
			USER_CONTROLLED,
		} trackingState;

		static void computeControlCB(SBHeliState *state, void *ctxt)
		{
			SBController * that = (SBController*)ctxt;
			that->computeControl(state);
		}

		void computeControl(SBHeliState * state)
		{
			// Superimpose random noise on control imput
			desRoll =  ((rand()/float(RAND_MAX))-.5) * CONTROL_NOISE + state->rcChannel[SB_RC_ROLL];
			desPitch = ((rand()/float(RAND_MAX))-.5) * CONTROL_NOISE + state->rcChannel[SB_RC_PITCH];
			if ((simple->state.mode.navigation == SB_NAV_CTRLLED)
					&& (simple->state.rcChannel[SB_RC_THROTTLE]>THROTTLE_TRIM_LIMIT)) {
				assert(!isnan(desRoll));
				assert(!isnan(desPitch));
				DEBUG(res = sbSimpleControl(simple,desRoll,
							desPitch,0,0 /* way and altitude is manual */));
			}
			logbuffer.push_back(LogPacket(simple->state));
		}

	public:
		SBController(SBApiSimpleContext *s) 
			: simple(s), sensorList(0), res(0)
		{
			sbSimpleDefaultContext(simple);
			trackingState = HELI_ON_GROUND;
			desRoll = desPitch = 0.0;
		}

		~SBController() {
		}


		int initialise(const std::string & devname) {
			res = 0;
			sbSimpleParseChannel(simple,devname.c_str(),NULL);

			simple->cmdTimeout = 2000;
			simple->ctrlTimeout = 1000;
			simple->rollCtrlMode = SB_CTRL_POS;
			simple->pitchCtrlMode = SB_CTRL_POS;
			simple->altCtrlMode = SB_CTRL_MANUAL;
			simple->yawCtrlMode = SB_CTRL_MANUAL;
			simple->commFreq = 50;
			simple->commContent = SBS_MODES | SBS_TIMESTAMP | SBS_RPY | SBS_BATTERY |
				SBS_GYRO | SBS_ACCEL | SBS_ALTITUDE | SBS_O_ALTITUDE | SBS_O_ATTITUDE
				| SBS_HRANGES | SBS_CHANNELS ;

			simple->stateFunc = computeControlCB;
			simple->userData = this;

			CRITICAL(res = sbSimpleInitialise(simple));
			sbLockCommunication(&simple->control);
			CRITICAL(res = sbConfigureAckMode(&simple->control,0));
			sbUnlockCommunication(&simple->control);
			printf("Channel connected, continuing\n");

			return res;
		}

		int terminate() {
			res = 0;
			DEBUG(res = sbSimpleTerminate(simple));
			return res;
		}


		double now() {
			struct timeval tv;
			gettimeofday(&tv,NULL);
			return tv.tv_sec + 1e-6*tv.tv_usec;
		}

		int joyctrl() {
			// double timeOnGround = -1;
			DEBUG(res = sbSimpleReachNavState(simple,SB_NAV_CTRLLED,30.0));
			while (1) {
				if ((*simple->endP)) {
					break;
				}
				if (simple->state.errorFlags) {
					printf("An error has been detected on the PIC: %02X\n",
							simple->state.errorFlags);
					DEBUG(res = sbSimpleReachNavState(simple,SB_NAV_STOP,30.0));
					break;
				}
				switch (trackingState) {
					case HELI_ON_GROUND:
						if (simple->state.rcChannel[SB_RC_THROTTLE]>THROTTLE_TRIM_LIMIT) {
							DEBUG(res = sbSimpleReachNavState(simple,SB_NAV_CTRLLED,30.0));
							trackingState = USER_CONTROLLED;
						}
						break;
					case USER_CONTROLLED:
						if (simple->state.rcChannel[SB_RC_THROTTLE]<THROTTLE_TRIM_LIMIT) {
							DEBUG(res = sbSimpleReachNavState(simple,SB_NAV_STOP,30.0));
							trackingState = HELI_ON_GROUND;
						}
					default :
						break;
				}

				usleep(50000);  // 20Hz

			}
			return 0;
		}

		void savelog(const std::string & fname) {
			FILE * fp = NULL;
			int fd = -1;
			unsigned int i;
			fp = fopen(fname.c_str(),"w");
			if (fp == NULL) {
				char ftemplate[]="/tmp/savedlog.XXXXXX";
				fprintf(stderr,"Impossible to open \"%s\" for writing\n",fname.c_str());
				fd = mkstemp(ftemplate); fp = fdopen(fd,"w");
				fprintf(stderr,"Writing output to \"%s\"\n",ftemplate);
			}
			for (i=0;i<logbuffer.size();i++) {
				logbuffer[i].serialize(fp);
			}
			fclose(fp);
		}

};


int main(int argc, const char *argv[])
{
	const SBVersionStatus * compiled = sbGetCompiledVersion(); 
	unsigned int objSizeStatus = sbValidateObjectSize(compiled);
	printf("Object size status: %04x\n",objSizeStatus);
	assert(objSizeStatus == 0);

	char flogname[128]; 
	sprintf(flogname,"log-%08ld.txt",(unsigned long)time(NULL));

	SBApiSimpleContext simple;
	SBController api(&simple);


	int res;
	CRITICAL(res = api.initialise((argc<2)?("localhost"):(argv[1])));

	DEBUG(res = api.joyctrl());

	DEBUG(res = api.terminate());

	printf("Saving log\n");
	api.savelog(flogname);
	printf("Saved log\n");
	return 0;
}

		
