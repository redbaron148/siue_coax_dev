/**************************************************************************/
/* FILE:    MAIN.C                                                        */
/* AUTHOR:  SAMIR BOUABDALLAH , Gilles Caprari                            */
/* DATE:    13.10.2008                                                    */
/* HISTORY:                                                               */
/*          13.10.2008 - Initial Release; Samir Bouabdallah               */
/*          17.10.2008 - test all ports copying incoming msg to both out  */
/**************************************************************************/

/*this programm is a test for both UART 
*/

/* ===== STANDARD INCLUDES. ============================================= */
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
#include <uart/coax_uart.h>
//#include "e_bluetooth.h"

#define uart1_send_static_text(msg) { e_send_uart1_char(msg,sizeof(msg)-1); while(e_uart1_sending()); }


/* ===== CONFIGURATION BITS ============================================= */
//_FBS(BWRP_WRPROTECT_OFF);							// Boot segment
//_FSS(SSS_NO_SEC_CODE & SWRP_WRPROTECT_OFF);		// Secure segment
_FOSCSEL(FNOSC_PRI);								// Primary (XT, HS, EC) Oscillator
_FOSC(FCKSM_CSECME & OSCIOFNC_OFF  & POSCMD_HS);  
													// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
													// OSC2 Pin Function: OSC2 is Clock Output
													// Primary Oscillator Mode: XT Crystanl
_FWDT(FWDTEN_OFF);              					// Watchdog Timer Enabled/disabled by user software
//_FWDT(FWDTEN_OFF & WINDIS_OFF)					// Wdt
//_FPOR(FPWRT_PWR128);								// POR (Power On Reset)


/* ===== DEFINITIONS. =================================================== */

/* ===== GLOBALS. ======================================================= */

// General Globals. --------------------------------------------------------
//int lixo[1000];		// http://forum.microchip.com/tm.aspx?m=230030&mpage=1&key=&#230030

/* ===== MAIN. ========================================================== */

int main(void) {

	// Locals General. -----------------------------------------------------
	char	command[5], response[50];

	// Init Oscillator. ----------------------------------------------------
	InitOscillator();	// Initialize the PLL (also disables wdt)
	WaitMiliSec(50);
	// Init mcu ports ------------------------------------------------------
	init_port();    	// Initialize ports

	// Init UARTS. ---------------------------------------------------------	
	init_UART1();		// Initialize the serial communication (TTL / RS-232)
	init_UART2();		// Initialize the serial communication (TTL / RS-232)

	// Welcome Message-----------------------------------------------------

	LED_ORNG =1;
	uart1_send_static_text("\n\n\rWELCOME to CoaX UART test");	// Welcome msg
	uart1_send_static_text("\n\rPress any key to check");
	LED_ORNG=0;

	while(1)
	{	
		if ( e_getchar_uart1(&command[0]) ){
			LED_ORNG =1;
			command[1]=0;
			sprintf(response,"\n\rFrom U1:'%s'; %02X.", &command[0], command[0]);
			e_send_uart1_char(response,strlen(response));
			while(e_uart1_sending());
			e_send_uart2_char(response,strlen(response));
			while(e_uart2_sending());
			LED_ORNG=0;
		}

		if ( e_getchar_uart2(&command[0]) ){
			LED_ORNG =1;
			command[1]=0;
			sprintf(response,"\n\rFrom U2:'%s'; %02X", &command[0], command[0]);
			e_send_uart1_char(response,strlen(response));
			while(e_uart1_sending());
			e_send_uart2_char(response,strlen(response));
			while(e_uart2_sending());
			LED_ORNG=0;
		}
	}
}
