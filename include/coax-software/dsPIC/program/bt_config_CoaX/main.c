/**************************************************************************/
/* FILE:    MAIN.C                                                        */
/* AUTHOR:  SAMIR BOUABDALLAH , Gilles Caprari                            */
/* DATE:    13.10.2008                                                    */
/* HISTORY:                                                               */
/*          13.10.2008 - Initial Release; Samir Bouabdallah               */
/*          17.10.2008 - Bluetooth configuration routines                 */
/**************************************************************************/

/*this programm is an exemple how to used bluetooth library. It is inspired by the e-puck library
* You can comunicate through normal RS232 conncection (need cable) and test bluetooth function
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
#include "e_bluetooth.h"

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
	char message[50];	// Any message to send by UART1

	 char	command[20], response[50];
	 int	c;
	 int	i, version;

	// Init Oscillator. ----------------------------------------------------
	InitOscillator();	// Initialize the PLL (also disables wdt)
	WaitMiliSec(50);
	// Init mcu ports ------------------------------------------------------
	init_port();    	// Initialize ports

	// Init UARTS. ---------------------------------------------------------	
	init_UART1();		// Initialize the serial communication (TTL / RS-232)
	init_UART2();		// Initialize the serial communication (TTL / RS-232)

	LED_ORNG=0;
//	LED_ORNG =1;

	// Welcome Message-----------------------------------------------------

	uart1_send_static_text("\n\n\rWELCOME to CoaX Bluetooth configuration");	// Welcome msg
	uart1_send_static_text("\n\rPress H (return) for help");

	while(1)
	{	
	   i = 0;
		c=0;
	   do
	   {
	     	if (e_getchar_uart1(&command[i]))
			{	
				c=command[i];
	    		i++;
			}	
	   }
	   while (((char)c != '\n')&&((char)c != '\x0d'));
	   command[i]='\0';
		
	   switch (command[0]) 
	   { 
		case 'P':	e_bt_read_local_pin_number(message);
					sprintf(response,"\n\rPIN code = %s",message);
					break;
		case 'O':	sscanf(command,"O,%s\n",message);
					if(e_bt_write_local_pin_number(message))
						sprintf(response,"\n\rError writting PIN");
					else
						sprintf(response,"\n\rPIN code = %s",message);
					break;
		case 'M':	sscanf(command,"M,%s\n",message);
					if(e_bt_write_local_name(message))
						sprintf(response,"\n\rError writting Name");
					else
						sprintf(response,"\n\rFriendly name = %s",message);
					break;
		case 'S':	sscanf(command,"S,%s\n",message);
					if(e_bt_write_local_pin_number(message))
						sprintf(response,"\n\rError writting PIN");
					else
						sprintf(response,"\n\rPIN code = %s",message);
					e_send_uart1_char(response,strlen(response)); 
					while(e_uart1_sending());
					sprintf(command,"CoaX_%s",message);
					if(e_bt_write_local_name(command))
						sprintf(response,"\n\rError writting Name");
					else
						sprintf(response,"\n\rFriendly name = %s",command);
					break;
		case 'N':	e_bt_read_local_name(message);
					sprintf(response,"\n\rFriendly name = %s",message);
					break;
		case 'R':	version=e_bt_reset();
					sprintf(response,"\n\rReset ok Firmware = %d",version);
					break;
		case 'H':	uart1_send_static_text("\n\r \"M,Name\" Write Name for Friendly Bluetooth name");
					uart1_send_static_text("\n\r \"N\" Read actual Friendly Bluetooth name");
					uart1_send_static_text("\n\r \"O,#\" Write # PIN number");
					uart1_send_static_text("\n\r \"P\" Read actual PIN number"); 
					uart1_send_static_text("\n\r \"R\" Soft reset Bluetooth module");	
					uart1_send_static_text("\n\r \"S,#\" Write # PIN number and same time CoaX_#");	
					
					response[0]='\n';
					response[1]='\0';
					break;
	   	default:    sprintf(response,"\n\rz,Command not found");
	                 	break;
	   }
	   e_send_uart1_char(response,strlen(response));
	   while(e_uart1_sending());
	 }
}
