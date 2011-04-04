
#include "rc/rc_statemachine.h"
#include "control/control.h"
#include "configs/coax_ports.h"
#include "utils/utils.h"
#include "mouse/mouse.h"

static RC_CONTROL_STATE smState = RC_KILLED, smFlyMode = RC_MANUAL_FLY_DEFAULT;
static unsigned int lastRawThrottleTrim = 0;
static unsigned int numDelta=0;
static unsigned int timeInInter=0;

void RCInitStateMachine()
{
    smState = RC_INITIALIZING;
    numDelta = 0;
    lastRawThrottleTrim = 0;
    timeInInter = 0;
}

RC_CONTROL_STATE RCSMGetState() {
    return smState;
}

RC_CONTROL_STATE RCSMGetFlyMode() {
    if ((RC_ROLL < -0.5) && (RC_PITCH < -0.5)) {
        BuzzerBip(3,0);
        return RC_MANUAL_FLY_THRUST;
    } else if ((RC_ROLL < -0.5) && (RC_PITCH > 0.5)) {
        BuzzerBip(1,0);
        return RC_MANUAL_FLY_COUPLED;
    } else if ((RC_ROLL > 0.5) && (RC_PITCH > 0.5)) {
        BuzzerBip(1,0);
        return RC_MANUAL_FLY_DEFAULT;
    } else if ((RC_ROLL > 0.5) && (RC_PITCH < -0.5)) {
        BuzzerBip(2,0);
        return RC_MANUAL_FLY_RANGE;
    } else {
        if (mouse.sensor_available) {
            BuzzerBip(4,0);
            return RC_MANUAL_FLY_COAXSPEED;
        } else {
            BuzzerBip(1,0);
            return RC_MANUAL_FLY_DEFAULT;
        }
    }
}

static RC_CONTROL_STATE RCSMUpdateState_WK2401() {
	int updownevent = 0;
    // Detect basic transitions based on RC_THROTTLE_TRIM:
    // From manual: bringing the trim up and down in  less than max_time_in_inter is called
    // an "updownevent". This is detected by going first to a CONTROL_INTER
    // state
    // From manual: bringing the trim up for more than max_time_in_inter switch to
    // auto in this mode
    // From auto: bringing the trim down switch to manual, immediately
    switch (smState) {
        case RC_STOP:
        case RC_MANUAL_IDLE:
        case RC_MANUAL_FLY_READY:
        case RC_MANUAL_FLY_RANGE:
        case RC_MANUAL_FLY_THRUST:
        case RC_MANUAL_FLY_COUPLED:
        case RC_MANUAL_FLY_COAXSPEED:
            if ((RC_THROTTLE_TRIM > 0.8) && !timeInInter) {
                timeInInter = 1;
            } else if ((RC_THROTTLE_TRIM >= 0.2) && timeInInter) {
                timeInInter ++;
                if (timeInInter > control.params.max_time_in_inter) {
                    timeInInter = control.params.max_time_in_inter;
                }
            } else if (RC_THROTTLE_TRIM < 0.2) {
                if ((timeInInter>0) && (timeInInter < control.params.max_time_in_inter)) {
                    updownevent = 1;
                }
                timeInInter = 0;
            }
            break;
        default:
            timeInInter = 0;
            break;
    }

    switch (smState) {
        case RC_INITIALIZING:
            if (RCIsReady()) {
                LED_ORNG = 0;
                smState = RC_KILLED;
            } else {
                LED_ORNG = 1;
            }
            break;
        case RC_STOP:
            if (control.errors.LOW_POWER_DETECTED) {
                // Can't get out of stop once LOW_POWER_DETECTED has been
                // triggered
                break;
            }
            LED_RED = 0;
            if (RC_YAW_TRIM > 0.5) {
                smState = RC_KILLED;
            } else if (updownevent && (RC_THROTTLE<0.2)) {
                smState = RC_MANUAL_IDLE;
            } else if ((RC_THROTTLE_TRIM>0.8) && (timeInInter>=control.params.max_time_in_inter)) {
                smState = RC_AUTO;
                BuzzerBip(3,0);
            }
            break;
        case RC_AUTO:
            if (RC_YAW_TRIM > 0.5) {
                smState = RC_KILLED;
            } else if (RC_THROTTLE_TRIM < 0.2) {
                smState = RC_STOP;
                BuzzerBip(3,0);
            }
            break;
        case RC_MANUAL_IDLE:
            if (RC_YAW_TRIM > 0.5) {
                smState = RC_KILLED;
            } else if (control.flags.time_in_idle >= control.params.max_time_in_mode) {
                smState = RC_STOP;
            } else if (updownevent && (RC_THROTTLE < 0.2) && !control.errors.LOW_POWER_DETECTED
                    && (control.flags.time_in_idle > control.params.min_time_in_idle)) {
                smFlyMode = RCSMGetFlyMode();
                smState = RC_MANUAL_FLY_READY;
            }
            break;
        case RC_MANUAL_FLY_READY:
            if ((RC_ROLL>-0.1) && (RC_ROLL<0.1) && (RC_PITCH>-0.1) && (RC_PITCH<0.1)) {
                smState = smFlyMode;
            }
            // fall-through
        case RC_MANUAL_FLY_RANGE:
        case RC_MANUAL_FLY_THRUST:
        case RC_MANUAL_FLY_COUPLED:
        case RC_MANUAL_FLY_COAXSPEED:
            if (RC_YAW_TRIM > 0.5) {
                smState = RC_KILLED;
            } else if (updownevent) {
                smState = RC_MANUAL_IDLE;
            }
            break;
        case RC_KILLED:
            LED_RED = 1;
            if ((RC_THROTTLE<0.2) && (RC_THROTTLE_TRIM<0.2) && (RC_YAW_TRIM<0.5)) {
                smState = RC_STOP;
            }
            break;
        default:
            // We should not come here, but just in case...
            smState = RC_KILLED;
            break;
    }
    return smState;
}

static RC_CONTROL_STATE RCSMUpdateState_WK2402() {
    int minustwodelta = 0;
    int plustwodelta = 0;
    int killCondition = 0;
    if (numDelta) {
        timeInInter++;
        if (timeInInter > control.params.max_time_in_inter) {
            timeInInter = control.params.max_time_in_inter;
            numDelta = 0;
        } 
    }
    if (WK2402_RAW_THROTTLE_TRIM > lastRawThrottleTrim) { // Maybe add an epsilon
        numDelta += 1;
        if (numDelta > 1) {
            plustwodelta = 1;
            numDelta = 0;
        }
        timeInInter = 0;
    }
    if (WK2402_RAW_THROTTLE_TRIM < lastRawThrottleTrim) { // Maybe add an epsilon
        numDelta -= 1;
        if (numDelta < -1) {
            minustwodelta = 1;
            numDelta = 0;
        }
        timeInInter = 0;
    }
    lastRawThrottleTrim = WK2402_RAW_THROTTLE_TRIM;

    killCondition = (RC_YAW > 0.8) && (RC_THROTTLE < 0.1);
    switch (smState) {
        case RC_INITIALIZING:
            if (RCIsReady()) {
                LED_ORNG = 0;
                smState = RC_KILLED;
            } else {
                LED_ORNG = 1;
            }
            break;
        case RC_STOP:
            if (control.errors.LOW_POWER_DETECTED) {
                // Can't get out of stop once LOW_POWER_DETECTED has been
                // triggered
                break;
            }
            LED_RED = 0;
            if (killCondition) {
                smState = RC_KILLED;
            } else if (RC_YAW_TRIM < -0.85) {
                smState = RC_AUTO;
                BuzzerBip(3,0);
            } else if (plustwodelta && (RC_THROTTLE < 0.1)) {
                smState = RC_MANUAL_IDLE;
            }
            break;
        case RC_AUTO:
            if (killCondition) {
                smState = RC_KILLED;
            } else if (RC_YAW_TRIM > -0.8) {
                smState = RC_STOP;
                BuzzerBip(3,0);
            }
            break;
        case RC_MANUAL_IDLE:
            if (killCondition) {
                smState = RC_KILLED;
            } else if (minustwodelta) {
                smState = RC_STOP;
            } else if (control.flags.time_in_idle >= control.params.max_time_in_mode) {
                smState = RC_STOP;
            } else if (plustwodelta && (RC_THROTTLE < 0.1) 
                    && (control.flags.time_in_idle > control.params.min_time_in_idle)
                    && !control.errors.LOW_POWER_DETECTED) {
                smState = RC_MANUAL_FLY_READY;
                smFlyMode = RCSMGetFlyMode();
            }
            break;
        case RC_MANUAL_FLY_READY:
            if ((RC_ROLL>-0.1) && (RC_ROLL<0.1) && (RC_PITCH>-0.1) && (RC_PITCH<0.1)) {
                smState = smFlyMode;
            }
            // fall-through
        case RC_MANUAL_FLY_RANGE:
        case RC_MANUAL_FLY_THRUST:
        case RC_MANUAL_FLY_COUPLED:
        case RC_MANUAL_FLY_COAXSPEED:
            if (killCondition) {
                smState = RC_KILLED;
            } else if (minustwodelta) { // Maybe only if throttle < 0.2?
                smState = RC_MANUAL_IDLE;
            }
            break;
        case RC_KILLED:
            LED_RED = 1;
#if 0
            if ((RC_YAW<-0.9) && (RC_THROTTLE<0.1)  // Touch the right-bottom corner
                    && (RC_THROTTLE_TRIM<0.9) // To have space to make plustwodeltas
                    && (RC_YAW_TRIM>-0.8)) {   // To be sure we're back in manual
                smState = RC_STOP;
            }
#else
            if ((RC_YAW<-0.9) && (RC_THROTTLE<0.1)  // Touch the right-bottom corner
                    && (RC_THROTTLE_TRIM<0.9)) { // To have space to make plustwodeltas
                if (RC_YAW_TRIM < -0.85) {
                    BuzzerBip(3,0);
                    smState = RC_AUTO;
                } else {
                    smState = RC_STOP;
                }
            }
#endif
            break;
        default:
            // We should not come here, but just in case...
            smState = RC_KILLED;
            break;
    }
    return smState;
}

void RCSMSwitchToIdle() {
    switch (smState) {
        case RC_INITIALIZING:
        case RC_STOP:
        case RC_AUTO:
        case RC_MANUAL_IDLE:
        case RC_KILLED:
            break;
        case RC_MANUAL_FLY_READY:
        case RC_MANUAL_FLY_RANGE:
        case RC_MANUAL_FLY_THRUST:
        case RC_MANUAL_FLY_COUPLED:
        case RC_MANUAL_FLY_COAXSPEED:
            smState = RC_MANUAL_IDLE;
            break;
        default:
            // We should not come here, but just in case...
            smState = RC_KILLED;
            break;
    }
}


RC_CONTROL_STATE RCSMUpdateState() {
    switch (RCGetType()) {
        case RC_WK2401:
            return RCSMUpdateState_WK2401();
        case RC_WK2402:
            return RCSMUpdateState_WK2402();
        default:
            break;
    }
    return smState;
}


