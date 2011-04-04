#ifndef SB_UTILS_H
#define SB_UTILS_H

#include <com/sbconst.h>
#include <com/sbchannel.h>
#include <com/sbcommloop.h>

void error_display(unsigned int errorLevel, const char * message);

void reset(void);

int updateControlParams(int setParams, SBControlParametersMessage *params);

int updateTrimMode(int setMode, SBTrimModeMessage *mode);

void setLightSensor(unsigned short percent);

int configureBluetooth(const char code[4], const char name[16]);

double now();

void setPeriodicState(unsigned int source, unsigned int period_ms);


#endif // SB_UTILS_H
