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
#include <sharp/sharp.h>




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


int main(void) {

	// Locals General. -----------------------------------------------------
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


    broadcast_message("Entering main loop\n");
	// Init BUZZER. ----------------------------------------------------------
	BuzzerBip(3,1);		// Make x bips of the Buzzer, blocking
    while (1) {
        __delay32(5*_10MILLISEC);
        updateSHARP_FRONT();
        updateSHARP_LEFT();
        updateSHARP_RIGHT();
        updateSHARP_BACK();
        sprintf(message, "Range %6d %6d %6d %6d\n",
                sharp_int.front, sharp_int.left, 
                sharp_int.right, sharp_int.back);
        broadcast_message(message);
    }

    return 0;
	
}	// End of main



