/*********************************************************************
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
*********************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef WIN32
#define sscanf sscanf_s
#define strcpy strcpy_s
#define strncpy strncpy_s

double remainder(double x, double mod) {
	int n = (int)floor(x/mod);
	return x - (n*mod);
}

#endif

#include <string>
#include <vector>

#include "com/sbapi.h"
#include "com/sbsimple.h"

#include "MenuManager.h"

#define D2R(X) ((X)*M_PI/180.0)
#define R2D(X) ((X)*180.0/M_PI)
#define MAX(X,Y) (((X)<(Y))?(Y):(X))
#define MIN(X,Y) (((X)<(Y))?(X):(Y))

//#define DEBUG(c) printf("Executing "#c"\n")
//#define DEBUG(c) res=0;printf("Executing "#c"\n");c;printf("Result %d\n",res)
#define DEBUG(c) res=0;c;if (res) printf("Result of "#c": %d\n",res)

static int end = 0;

class SBApiManager
{
	protected:
		SBApiSimpleContext *simple;
		unsigned long sensorList;
		int res;
	public:
		SBApiManager(SBApiSimpleContext *s) : simple(s), sensorList(0), res(0) {}
		~SBApiManager() {}

		int initialise(const std::vector<std::string> & args) {
			// DEBUG(sbSimpleDefaultContext(simple));
			res = 0;
			sbSimpleDefaultContext(simple);
			if (args.size() > 1) {
				sbSimpleParseChannel(simple,args[1].c_str(),args[2].c_str());
			}
			simple->initNavState = SB_NAV_STOP;
			simple->cmdTimeout = 30000;
			simple->ctrlTimeout = 5000;
			simple->sensorList = &sensorList;
			simple->rollCtrlMode = SB_CTRL_POS;
			simple->pitchCtrlMode = SB_CTRL_POS;
			simple->oaMode = SB_OA_NONE;

			DEBUG(res = sbSimpleInitialise(simple));
			if (res == 0) {
				printf("Channel connected, continuing\n");
			}

			return res;
		}

		int terminate(const std::vector<std::string> & args) {
			res = 0;
			DEBUG(res = sbSimpleTerminate(simple));
			return res;
		}

		int restart(const std::vector<std::string> & args) {
			res  = 0;
			DEBUG(res = sbConnect(&simple->control));
			return res;
		}

		int quit(const std::vector<std::string> & args) {
			end ++;
			return 0;
		}

		int string(const std::vector<std::string> & args) {
			std::string s;
			res = 0;
			for (unsigned int i=1;i<args.size();i++) {
				s += args[i];
				s += " ";
			}
			if (s.size() > SB_STRING_MESSAGE_LENGTH) {
				s = s.substr(0,SB_STRING_MESSAGE_LENGTH);
			}
			printf("Sending string '%s'\n",s.c_str());
			DEBUG(sbLockCommunication(&simple->control));
			DEBUG(res = sbSendString(&simple->control,s.c_str()));
			DEBUG(sbUnlockCommunication(&simple->control));
			return res;
		}


		int keepalive(const std::vector<std::string> & args) {
			res = 0;
			DEBUG(sbLockCommunication(&simple->control));
			DEBUG(res = sbKeepAlive(&simple->control));
			DEBUG(sbUnlockCommunication(&simple->control));
			return res;
		}

		int idle(const std::vector<std::string> & args) {
			res = 0;
			DEBUG(res = sbSimpleReachNavState(simple,SB_NAV_IDLE,20.0));
			return res;
		}

		int reset(const std::vector<std::string> & args) {
			res = 0;
			DEBUG(sbLockCommunication(&simple->control));
			DEBUG(res = sbResetDSPIC(&simple->control));
			DEBUG(sbUnlockCommunication(&simple->control));
			return res;
		}

		int trimmode(const std::vector<std::string> & args) {
			struct SBTrimMode mode;
			res = 0;
			DEBUG(sbLockCommunication(&simple->control));
			DEBUG(res = sbGetTrimMode(&simple->control,&mode));
			DEBUG(sbUnlockCommunication(&simple->control));
			if (res == 0) {
				printf("Mode: %s roll %f pitch %f\n",
						sbTrimModeString(mode.trimMode),mode.rollTrim,mode.pitchTrim);
			}
			return res;
		}

		int ctrlparm(const std::vector<std::string> & args) {
			struct SBControlParameters params;
			res = 0;
			DEBUG(sbLockCommunication(&simple->control));
			DEBUG(res = sbGetControlParameters(&simple->control,&params));
			DEBUG(sbUnlockCommunication(&simple->control));
			if (res == 0) {
				printf("Control Paramters:\n\t Altitude Kh %.3f P %.3f I %.3f D %.3f\n",
						params.baseThrust,params.altitudeKp,params.altitudeKi,params.altitudeKd);
				printf("\t Yaw Offset %.3f P %.3f I %.3f D %.3f\n",
						params.yawOffset,params.yawKp,params.yawKi,params.yawKd);
			}
			return res;
		}

		int setkh(const std::vector<std::string> & args) {
			double kh;
			struct SBControlParameters params;
			res = 0;
			MenuManager::checkArguments(args,1,1);
			DEBUG(sbLockCommunication(&simple->control));
			DEBUG(res = sbGetControlParameters(&simple->control,&params));
			DEBUG(sbUnlockCommunication(&simple->control));
			if (res == 0) {
				printf("Initial state\n");
				printf("Control Paramters:\n\t Altitude Kh %.3f P %.3f I %.3f D %.3f\n",
						params.baseThrust,params.altitudeKp,params.altitudeKi,params.altitudeKd);
				printf("\t Yaw Offset %.3f P %.3f I %.3f D %.3f\n",
						params.yawOffset,params.yawKp,params.yawKi,params.yawKd);
			}
			kh = params.baseThrust;
			sscanf(args[1].c_str(),"%le",&kh);
			DEBUG(sbLockCommunication(&simple->control));
			DEBUG(res = sbSetControlParameters(&simple->control,&params));
			DEBUG(res = sbGetControlParameters(&simple->control,&params));
			DEBUG(sbUnlockCommunication(&simple->control));
			if (res == 0) {
				printf("Configured state\n");
				printf("Control Paramters:\n\t Altitude Kh %.3f P %.3f I %.3f D %.3f\n",
						params.baseThrust,params.altitudeKp,params.altitudeKi,params.altitudeKd);
				printf("\t Yaw Offset %.3f P %.3f I %.3f D %.3f\n",
						params.yawOffset,params.yawKp,params.yawKi,params.yawKd);
			}
			return res;
		}

		int commstop(const std::vector<std::string> & args) {
			res = 0;
			DEBUG(sbLockCommunication(&simple->control));
			DEBUG(res = sbConfigureComm(&simple->control,SB_COM_ONREQUEST,0,0,0));
			DEBUG(sbUnlockCommunication(&simple->control));
			return res;
		}

		int commstart(const std::vector<std::string> & args) {
			int r[2];
			int freq;
			long unsigned int contents;
			res = 0;
			MenuManager::checkArguments(args,2,2);
			r[0] = sscanf(args[1].c_str(),"%d",&freq) == 1;
			if (args[2] == "all") {
				contents = SBS_ALL;
				r[1] = true;
			} else {
				r[1] = sscanf(args[2].c_str(),"%lX",&contents) == 1;
			}
			if (r[0] && r[1]) {
				DEBUG(sbLockCommunication(&simple->control));
				DEBUG(res = sbConfigureComm(&simple->control,SB_COM_CONTINUOUS,freq,5*freq,contents));
				DEBUG(sbUnlockCommunication(&simple->control));
			} else {
				printf("Usage: commstart <frequency> <all | content (Hex)>\n");
				return -1;
			}
			return res;
		}

		int reqstate(const std::vector<std::string> & args) {
			SBHeliState mystate;
			res = 0;
			DEBUG(sbLockCommunication(&simple->control));
			DEBUG(res = sbRequestState(&simple->control,SBS_ALL,&mystate));
			DEBUG(sbUnlockCommunication(&simple->control));
			DEBUG(sbStatePrint(stdout,&mystate));
			return res;
		}

		int state(const std::vector<std::string> & args) {
			res = 0;
			DEBUG(res = sbSimpleWaitState(simple,NULL,1.0));
			printf("Packet count: %ld\n",simple->packetcount);
			DEBUG(sbStatePrint(stdout,&simple->state));
			return res;
		}

		int sensors(const std::vector<std::string> & args) {
			res = 0;
			DEBUG(sbLockCommunication(&simple->control));
			DEBUG(res = sbGetSensorList(&simple->control,&sensorList));
			DEBUG(sbUnlockCommunication(&simple->control));
			sbContentPrint(stdout,sensorList);
			return res;
		}

		int verbose(const std::vector<std::string> & args) {
			unsigned int verbose = 1;
			int debug_channel = -1;
			res = 0;
			MenuManager::checkArguments(args,0,2);
			if (args.size() > 0) {
				sscanf(args[1].c_str(),"%d", &verbose);
			}
			if (args.size() > 1) {
				sscanf(args[2].c_str(),"%d", &debug_channel);
			}
			DEBUG(sbSimpleSetVerbose(simple,verbose,debug_channel));
			return res;
		}

		int monitor(const std::vector<std::string> & args) {
			double period = 1.0;
			res = 0;
			MenuManager::checkArguments(args,1,1);
			sscanf(args[1].c_str(),"%le", &period);
			if (period < 1e-3) period = 1.0;
			if (period > 10) period = 10;
			while (!*(simple->endP)) {
				DEBUG(sbSimpleKeepAlive(simple));
				DEBUG(sbSimpleWaitState(simple,NULL,1.0));
				printf("Packet count: %ld\n",simple->packetcount);
				DEBUG(sbStatePrint(stdout,&(simple->state)));
#ifdef WIN32
				Sleep((unsigned int)period);
#endif
#ifdef LINUX
				usleep((unsigned int)(period*1e6));
#endif
			}
			*(simple->endP) = 0;
			return 0;
		}

		int yaw(const std::vector<std::string> & args) {
			double angle = 0;
			res = 0;
			MenuManager::checkArguments(args,1,1);
			if (sscanf(args[1].c_str(), "%le", &angle) == 1) {
				DEBUG(sbSimpleWaitState(simple,NULL,1.0));
				DEBUG(res = sbSimpleControl(simple,0,0,D2R(angle),simple->state.zrange));
			} else {
				printf("Usage: yaw <angle in degrees>\n");
				return -1;
			}
			return res;
		}

		int pitch(const std::vector<std::string> & args) {
			double angle = 0;
			res = 0;
			MenuManager::checkArguments(args,1,1);
			if (sscanf(args[1].c_str(), "%le", &angle) == 1) {
				DEBUG(sbSimpleWaitState(simple,NULL,1.0));
				DEBUG(res = sbSimpleControl(simple,0,angle,0,simple->state.zrange));
			} else {
				printf("Usage: pitch <angle in degrees>\n");
				return -1;
			}
			return res;
		}

		int roll(const std::vector<std::string> & args) {
			double angle = 0;
			res = 0;
			MenuManager::checkArguments(args,1,1);
			if (sscanf(args[1].c_str(), "%le", &angle) == 1) {
				DEBUG(sbSimpleWaitState(simple,NULL,1.0));
				DEBUG(res = sbSimpleControl(simple,angle,0,0,simple->state.zrange));
			} else {
				printf("Usage: roll <angle in degrees>\n");
				return -1;
			}
			return res;
		}

		int alt(const std::vector<std::string> & args) {
			double z = 0;
			res = 0;
			MenuManager::checkArguments(args,1,1);
			if (sscanf(args[1].c_str(), "%le", &z) == 1) {
				DEBUG(sbSimpleWaitState(simple,NULL,1.0));
				DEBUG(res = sbSimpleControl(simple,0,0,0,z));
			} else {
				printf("Usage: altitude <height in meter>\n");
				return -1;
			}
			return res;
		}

		int nav(const std::vector<std::string> & args) {
			int navid = 0;
			res = 0;
			MenuManager::checkArguments(args,1,1);
			if (sscanf(args[1].c_str(), "%d", &navid) == 1) {
				DEBUG(res = sbSimpleReachNavState(simple,navid,30.0));
			} else {
				printf("Usage: nav <nav id>\n");
				return -1;
			}
			return res;
		}

		int raw(const std::vector<std::string> & args) {
			int r[4];
			double m1,m2,s1,s2;
			res = 0;
			MenuManager::checkArguments(args,4,4);
			r[0] = sscanf(args[1].c_str(),"%le",&m1) == 1;
			r[1] = sscanf(args[2].c_str(),"%le",&m2) == 1;
			r[2] = sscanf(args[3].c_str(),"%le",&s1) == 1;
			r[3] = sscanf(args[4].c_str(),"%le",&s2) == 1;
			if (r[0] && r[1] && r[2] && r[3]) {
				DEBUG(sbSimpleReachNavState(simple,SB_NAV_RAW,5.0));
				DEBUG(res = sbSimpleRawControl(simple,m1,m2,s1,s1));
			} else {
				printf("Usage: raw <m1> <m2> <s1> <s2>\n");
				return -1;
			}
			return res;
		}

		int rstable(const std::vector<std::string> & args) {
			res = 0;
			double Kp = +0.1;
			MenuManager::checkArguments(args,0,0);
			DEBUG(sbSimpleWaitState(simple,NULL,1.0));
			double currentyaw = simple->state.yaw;
			double currentz = simple->state.zrange;
			while (!*(simple->endP)) {
				DEBUG(sbSimpleWaitState(simple,NULL,1.0));
				double error = remainder(currentyaw - simple->state.yaw,360.);
				double control = Kp * error;
				DEBUG(res = sbSimpleControl(simple,0,0,control,currentz));
				printf("Y %.2f Y* %.2f Error: %.2f Control %.2f\n",simple->state.yaw, currentyaw,  error, control);
			}
			*(simple->endP) = 0;
			return res;
		}

		int rawprofile(const std::vector<std::string> & args) {
			int r[2];
			int m1,m2;
			res = 0;
			MenuManager::checkArguments(args,2,2);
			r[0] = sscanf(args[1].c_str(),"%d",&m1) == 1;
			r[1] = sscanf(args[2].c_str(),"%d",&m2) == 1;
			if (r[0] && r[1]) {
				DEBUG(sbLockCommunication(&simple->control));
				DEBUG(res = sbConfigureRawControl(&simple->control,m1,m2));
				DEBUG(sbUnlockCommunication(&simple->control));
			} else {
				printf("Usage: rawprofile <prof1> <prof2>\n");
				return -1;
			}
			return res;
		}

		int manual(const std::vector<std::string> & args) {
			int manual = 1;
			res = 0;
			MenuManager::checkArguments(args,0,1);
			int r;
			r = (sscanf(args[1].c_str(),"%d",&manual) == 1);
			DEBUG(sbLockCommunication(&simple->control));
			if (manual) {
				DEBUG(res = sbConfigureControl(&simple->control,
							SB_CTRL_MANUAL, SB_CTRL_MANUAL, 
							SB_CTRL_MANUAL, SB_CTRL_MANUAL | SB_CTRL_REL));
			} else {
				DEBUG(res = sbConfigureControl(&simple->control,
							SB_CTRL_POS, SB_CTRL_POS, 
							SB_CTRL_VEL, SB_CTRL_REL));
			}
			DEBUG(sbUnlockCommunication(&simple->control));
			return res;
		}

		int thrust(const std::vector<std::string> & args) {
			int thrust = 1;
			res = 0;
			MenuManager::checkArguments(args,0,1);
			int r;
			r = (sscanf(args[1].c_str(),"%d",&thrust) == 1);
			DEBUG(sbLockCommunication(&simple->control));
			if (thrust) {
				DEBUG(res = sbConfigureControl(&simple->control,
							SB_CTRL_MANUAL, SB_CTRL_MANUAL, 
							SB_CTRL_MANUAL, SB_CTRL_MANUAL | SB_CTRL_FORCE));
			} else {
				DEBUG(res = sbConfigureControl(&simple->control,
							SB_CTRL_POS, SB_CTRL_POS, 
							SB_CTRL_VEL, SB_CTRL_REL));
			}
			DEBUG(sbUnlockCommunication(&simple->control));
			return res;
		}


		int oamode(const std::vector<std::string> & args) {
			int r[2];
			int horizontal,vertical;
			res = 0;
			MenuManager::checkArguments(args,2,2);
			r[0] = sscanf(args[1].c_str(),"%d",&vertical) == 1;
			r[1] = sscanf(args[2].c_str(),"%d",&horizontal) == 1;
			if (r[0] && r[1]) {
				DEBUG(sbLockCommunication(&simple->control));
				DEBUG(res = sbConfigureOAMode(&simple->control,
							(vertical?SB_OA_VERTICAL:SB_OA_NONE) |
							(horizontal?SB_OA_HORIZONTAL:SB_OA_NONE)));
				DEBUG(sbUnlockCommunication(&simple->control));
			} else {
				printf("Usage: oamode <vertical> <horizontal>\n");
				return -1;
			}
			return res;
		}


		int timeout(const std::vector<std::string> & args) {
			int r[2];
			int cmd_timeout,ctrl_timeout;
			res = 0;
			MenuManager::checkArguments(args,2,2);
			r[0] = sscanf(args[1].c_str(),"%d",&cmd_timeout) == 1;
			r[1] = sscanf(args[2].c_str(),"%d",&ctrl_timeout) == 1;
			if (cmd_timeout < 0) cmd_timeout = 0xFFFF;
			if (ctrl_timeout < 0) ctrl_timeout = 0xFFFF;
			if (r[0] && r[1]) {
				DEBUG(sbLockCommunication(&simple->control));
				DEBUG(res = sbConfigureTimeout(&simple->control,cmd_timeout,ctrl_timeout));
				DEBUG(sbUnlockCommunication(&simple->control));
			} else {
				printf("Usage: timeout <command> <control>\n");
				return -1;
			}
			return res;
		}

};


#define ADD(name,help) \
	menu.add(#name,new MenuItemMember<SBApiManager>(&api,&SBApiManager::name,help))

int main(int argc, const char *argv[])
{
	std::vector<std::string> args;
	int i,res;
	SBApiSimpleContext simple;
	SBApiManager api(&simple);
	MenuManager menu;
	ADD(quit,"Quit current progam");
	ADD(verbose,"[bool] set control loop verbosity");
	ADD(reset,"Reset the micro-controller");
	ADD(string,"<text...> Send a string to the micro-controller");
	ADD(sensors,"Request and display the sensor list");
	ADD(oamode,"<vertical> <horizontal> Configure the obstacle avoidance mode");
	ADD(keepalive,"Send a keepalive message");
	ADD(restart,"Restart the connection, after a timeout for instance");
	ADD(idle,"Reach the idle mode");
	ADD(state,"Print out the current state");
	ADD(reqstate,"Request and Print out the current state");
	ADD(monitor,"<period> Print the system state every <period> seconds");
	ADD(roll,"<angle in degree> Reach the demanded roll angle");
	ADD(pitch,"<angle in degree> Reach the demanded pitch angle");
	ADD(yaw,"<angle in degree> Reach the demanded yaw angle");
	ADD(alt,"<height in meter> Reach the demanded altitude");
	ADD(manual, "[<0/_1_>] Switch to manual mode, with altitude control (or from)");
	ADD(thrust, "[<0/_1_>] Switch to manual mode, with thrust control (or from)");
	ADD(nav, "<nav id> Reach demanded navigation state");
	ADD(raw, "<m1> <m2> <s1> <s2> Send some raw command");
	ADD(rstable, "Stabilise yaw position");
	ADD(ctrlparm, "Get current control parameters");
	ADD(setkh, "<value> Set current control parameters: KH");
	ADD(trimmode, "Get current trim parameters");
	ADD(rawprofile, "<m1> <m2> Set the speed profiles");
	ADD(timeout, "<command> <control> Set the timeout parameters");
	ADD(commstop, "Stop continuous state transmission");
	ADD(commstart, "<frequency> <all | contents (Hex)> Start continuous state transmission");

	for(i=0;i<argc;i++) args.push_back(std::string(argv[i]));
	DEBUG(res = api.initialise(args));
	if (res) return -1;


	// SBSerialisedMessage sm;
	while (!end && !*simple.endP && !menu.endOfFile()) {
		menu.runOnce();
	}

	DEBUG(res = api.terminate(args));

	return 0;
}

		
