/**************************************************************************/
/* FILE:    MAIN.C                                                        */
/* AUTHOR:  SAMIR BOUABDALLAH                                             */
/* DATE:    13.10.2008                                                    */
/* HISTORY:                                                               */
/*          13.10.2008 - Initial Release; Samir Bouabdallah               */
/**************************************************************************/


/* ===== STANDARD INCLUDES. ============================================= */

#include <libpic30.h> // for delay32
#include <p33FJ256GP506.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <uart.h>


/* ===== CUSTOM MADE INCLUDES. ========================================== */

#include <configs/coax_ports.h>
#include <configs/init_port.h>
#include <utils/utils.h>
#include <led/coax_led.h>
#include <configs/coax_config.h>
#include <analog/analog.h>
#include <uart/coax_uart.h>
#include <rc/coax_rc.h>
#include <rc/rc_statemachine.h>
#include <control/control.h>
#include <i2c/e_I2C_protocol.h>
#include <imu/imu.h>
#include <imu/imu_ffp.h>
#include <spi/spi.h>




/* ===== CONFIGURATION BITS ============================================= */


_FBS(BWRP_WRPROTECT_OFF);							// Boot segment
_FSS(SSS_NO_SEC_CODE & SSS_NO_FLASH & SWRP_WRPROTECT_OFF);		// Secure segment

_FOSCSEL(FNOSC_PRI);								// Primary (XT, HS, EC) Oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF  & POSCMD_HS);  
// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: XT Crystanl
_FWDT(FWDTEN_OFF & WINDIS_OFF)						// Watchdog Timer
_FPOR(FPWRT_PWR128);								// POR (Power On Reset)


/* ===== DEFINITIONS. =================================================== */

/* ===== GLOBALS. ======================================================= */

// General Globals. --------------------------------------------------------
//int lixo[1000];		// http://forum.microchip.com/tm.aspx?m=230030&mpage=1&key=&#230030

/* ===== MAIN. ========================================================== */

void broadcast_message(const char * message) {
	while(BusyUART2());		// Wait until previous transmission is finished
	putsUART2 ((unsigned int *)message);
	while(BusyUART2());		// Wait until previous transmission is finished

	while(BusyUART1());		// Wait until previous transmission is finished
	putsUART1 ((unsigned int *)message);
	while(BusyUART1());		// Wait until previous transmission is finished
}

#define IMU_USE_FFP
#ifdef IMU_USE_FFP
#define updateIMUData updateIMUData_ffp
#endif

const char * string_of_control_type(mode) {
    switch (mode & 0x0F) {
        case CONTROL_RAW: return " RAW";
        case CONTROL_STOP: return "STOP";
        case CONTROL_IDLE: return "IDLE";
        case CONTROL_FLY: return " FLY";
        default : return "----";
    }
}

const char * string_of_control_mode(mode) {
    switch (mode & 0xF0) {
        case CONTROL_MANUAL: return "MANUAL";
        case CONTROL_INTER: return " INTER";
        case CONTROL_AUTO: return " AUTO ";
        default : return "------";
    }
}

const char * string_of_rc_state(state) {
    switch (state) {
        case RC_KILLED: return " KILLED";
        case RC_STOP: return "  STOP ";
        case RC_AUTO: return "  AUTO ";
        case RC_MANUAL_IDLE: return "  IDLE ";
        case RC_MANUAL_FLY_READY: return "FLY RDY";
        case RC_MANUAL_FLY_THRUST: return "FLY THR";
        case RC_MANUAL_FLY_RANGE: return "FLY RNG";
        case RC_MANUAL_FLY_COAXEYE: return "FLY CXE";
        default: return "-------";
    }
}



int main(void) {

	// Locals General. -----------------------------------------------------
	char imu_board_id[9] = {0,0,0,0, 0,0,0,0, 0};
	char message[1024];	// Any message to send by UART1
	InitOscillator();			// Initialize the PLL (also disables wdt)
	__delay32(_50MILLISEC);

	// Init mcu ports ------------------------------------------------------

	init_port();    	// Initialize ports
	LED_ORNG =0;
	LED_RED = 1;

	// Init UARTS. ---------------------------------------------------------

	init_UART1();		// Initialize the serial communication (TTL / RS-232)
	init_UART2();
    broadcast_message("RC Testing program\n");

	// Init Analog Channels. -----------------------------------------------

	analog_initAnalog();	// Init the ADC module

	// Init SPI. ---------------------------------------------------------

	init_SPI();

	// Init I2C. ---------------------------------------------------------

	e_i2cp_init();
	e_i2cp_enable();
	__delay32(_50MILLISEC);
	e_i2c_write(0x00);	// dummy byte to get the START bit on the bus (believe it or not!)


	// Init RC. ----------------------------------------------------------
	initControlVariables(NULL);

    broadcast_message("Initialising RC\n");
	LED_ORNG =1;
	LED_RED = 1;

	BuzzerBip(1,1);		// Make x bips of the Buzzer (blocking)
	RCInitReception();
    // RCSetType(RC_WK2401);
    RCSetType(RC_WK2402);
    
    broadcast_message("Initialising IMU\n");
	// Init BUZZER. ----------------------------------------------------------
	__delay32(_200MILLISEC);  // Wait for the IMU to boot

	while (INT_IMU==0)		// Wait for the IMU to boot
	{
		FlashORNG ();						// Flash the LED	
	}
	read_imu_version(imu_board_id); imu_board_id[8] = 0;
	
    broadcast_message("Entering main loop\n");
	// Init BUZZER. ----------------------------------------------------------
	BuzzerBip(3,1);		// Make x bips of the Buzzer, blocking
	InitLoopTimer(control.params.T_ctrl_ms);// Initialize & Enable control loop timer (20ms; 50Hz)
	LED_RED = 0;
    while (1) {
        __delay32(5*_10MILLISEC);
        sprintf(message, "RC %+.2f %+.2f %+.2f %.2f %.2f %+.2f %+.2f %+.2f %s CTRL %02X %s %s\n", 
                RC_THROTTLE, RC_YAW, RC_ROLL, RC_PITCH,
                RC_THROTTLE_TRIM, RC_YAW_TRIM, RC_ROLL_TRIM, RC_PITCH_TRIM,
                string_of_rc_state(RCSMGetState()),
                control.flags.CONTROL_MODE,
                string_of_control_mode(control.flags.CONTROL_MODE),
                string_of_control_type(control.flags.CONTROL_MODE));
        broadcast_message(message);
    }

    return 0;
	
}	// End of main



