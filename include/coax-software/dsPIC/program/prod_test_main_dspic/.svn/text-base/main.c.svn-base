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

#include <analog/analog.h>
#include <spi/spi.h>
#include <configs/coax_ports.h>
#include <configs/init_port.h>
#include <utils/utils.h>
#include <led/coax_led.h>
#include <configs/coax_config.h>
#include <i2c/e_I2C_protocol.h>
#include <imu/imu.h>
#include <imu/imu_ffp.h>
#include <uart/coax_uart.h>
#include <rc/coax_rc.h>

#include "e_bluetooth.h"



/* ===== CONFIGURATION BITS ============================================= */
//_FBS(BWRP_WRPROTECT_OFF);							// Boot segment
//_FSS(SSS_NO_SEC_CODE & SSS_NO_FLASH & SWRP_WRPROTECT_OFF);		// Secure segment

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

void broadcast_message(const char * message) {
	while(BusyUART2());		// Wait until previous transmission is finished
	putsUART2 ((unsigned int *)message);
	while(BusyUART2());		// Wait until previous transmission is finished

	while(BusyUART1());		// Wait until previous transmission is finished
	putsUART1 ((unsigned int *)message);
	while(BusyUART1());		// Wait until previous transmission is finished
}

void debug_message(const char * message) {
	// Used when configuring bluetooth
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
	char imu_board_id[9] = {0,0,0,0, 0,0,0,0, 0};
	char message[1024];	// Any message to send by UART1
	int version;
	InitOscillator();			// Initialize the PLL (also disables wdt)
	__delay32(_50MILLISEC);

	// Init mcu ports ------------------------------------------------------

	init_port();    	// Initialize ports
	LED_ORNG =0;
	LED_RED = 1;

	// Init UARTS. ---------------------------------------------------------

	init_UART1();		// Initialize the serial communication (TTL / RS-232)
	init_UART2();

#if 0
	// Initialise Bluetooth Channels. -----------------------------------------------
	debug_message("Initialising BLUETOOTH\n\r");
	if(e_bt_write_local_pin_number("0023"))
		debug_message("Error writting PIN\n\r");
	else
		debug_message("PIC code = 0023\n\r");

	if(e_bt_write_local_name("CoaX_0023"))
		debug_message("Error writting Name\n\r");
	else
		debug_message("Friendly name = Coax_0023\n\r");
	version=e_bt_reset();
	sprintf(message,"Reset ok Firmware = %d\n\r",version);
	debug_message(message);
#endif


	// Init Analog Channels. -----------------------------------------------
	broadcast_message("Initialising ANALOG\n\r");

	analog_initAnalog();	// Init the ADC module

	// Init SPI. ---------------------------------------------------------
	broadcast_message("Initialising SPI\n\r");

	init_SPI();

	broadcast_message("Initialising I2C\n\r");
	// Init I2C. ---------------------------------------------------------

	e_i2cp_init();
	e_i2cp_enable();
	__delay32(_50MILLISEC);
	e_i2c_write(0x00);	// dummy byte to get the START bit on the bus (believe it or not!)


	// Init LED. ----------------------------------------------------------

	LED_ORNG =1;
	LED_RED = 1;
    //
	// Init RC. ----------------------------------------------------------

	RCInitReception(0);
    RCSetType(RC_WK2402);

	// Init BUZZER. ----------------------------------------------------------

	BuzzerBip(1,1);		// Make x bips of the Buzzer (blocking)
	broadcast_message("Waiting for IMU\n\r");
	__delay32(_200MILLISEC);  // Wait for the IMU to boot
	while (INT_IMU==0) {FlashORNG();}		
	broadcast_message("IMU is booted\n\r");

	read_imu_version(imu_board_id); imu_board_id[8] = 0;
	sprintf(message,"IMU Is Ready: %s\n\r%c[2J",imu_board_id,27);
	broadcast_message(message);

	LED_RED = 0;
	while (1) {
		LED_ORNG = 1 - LED_ORNG;
		__delay32(5*_10MILLISEC);
		updateIMUData(IMU_UPDATE_ALL);
		updateBatteryVoltage(); 
		sprintf(message, "%c[2J%c[;HBoard %s\n\rEuler %+6.2f %+6.2f %+6.2f\n\r"
				"Accel %+6.2f %+6.2f %+6.2f\n\rGyros %+6.2f %+6.2f %+6.2f\n\r"
				"Mag   %+6.2f %+6.2f %+6.2f\n\rPress %+6.2f Altit %+6.2f \n\r"
				"Volt  %+6.2f\n\r", 27,27,imu_board_id,
				imuData.euler[0],imuData.euler[1],imuData.euler[2],
				imuData.accel[0],imuData.accel[1],imuData.accel[2],
				imuData.gyro[0],imuData.gyro[1],imuData.gyro[2],
				imuData.mag[0],imuData.mag[1],imuData.mag[2],
				imuData.pressure,imuData.altitude,voltage_level_filt);
		broadcast_message(message);
        if (RCIsReady()) {
            sprintf(message,"RC    ready -- throttle %.2f\n\r",RC_THROTTLE);
            broadcast_message(message);
        } else {
            broadcast_message("RC    scanning\n\r");
        }
	}
	
}	// End of main


