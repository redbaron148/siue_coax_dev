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
#ifndef SB_VERSION_H
#define SB_VERSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <com/sbconst.h>

/* Type representation of the version information, maps directly to the message
 * type */
typedef struct {
	/**
	 * Request the version information from the target system:
	 * apiVersion is the version of the communication API. It should be the same on
	 * both end of the communication link
	 * controllerVersion is the version of the controller running on the target
	 * platform. 
	 * compileTime is the textual representation of the compile time, obtained from
	 * the preprocessor macro __DATE__
	 * */
	unsigned short apiVersion;
	unsigned short controllerVersion;
	char imuVersion[SB_COMPILE_TIME_LENGTH];
	char compileTime[SB_COMPILE_TIME_LENGTH];

	/** 
	 * These will be used to check that the compile options in the library
	 * are the same as in the linking program
	 **/
	unsigned short sizeOf_SBApiSimpleContext;
	unsigned short sizeOf_SBControlContext;
	unsigned short sizeOf_SBHeliState;
	unsigned short sizeOf_SBHeliStateRaw;
} SBVersionStatus;

void sbInitialiseVersion(unsigned short controllerVersion);
void sbSetIMUVersion(const char * imuVersion);

const SBVersionStatus * sbGetCompiledVersion(void);

#ifdef SBC_HAS_IO
#include <stdio.h>
void sbPrintVersion(FILE * fp, const SBVersionStatus *version);
#endif

#include <com/sbstate.h>
#ifndef PIC30
#include <com/sbapi.h>
#include <com/sbsimple.h>
#else
struct SBControlContext {};
typedef int SBHeliState;
typedef int SBApiSimpleContext;
#endif
/** Return 0 on success, error code on failure 
 * Must be inlined to take the latest typedefs **/
/* const SBVersionStatus * compiled = sbGetCompiledVersion(); */
#define sbValidateObjectSize(compiled) (0 +                                            \
	((compiled->sizeOf_SBHeliStateRaw == sizeof(SBHeliStateRaw))?0:1) +                \
	((compiled->sizeOf_SBHeliState == sizeof(SBHeliState))?0:2) +                      \
	((compiled->sizeOf_SBControlContext == sizeof(struct SBControlContext))?0:4) +     \
	((compiled->sizeOf_SBApiSimpleContext == sizeof(SBApiSimpleContext))?0:8) +        \
	0 )

#ifdef __cplusplus
}
#endif


#endif /* SB_VERSION_H */
