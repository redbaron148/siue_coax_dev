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

#define IMU_USE_FFP
#ifdef IMU_USE_FFP
#define updateIMUData updateIMUData_ffp
#endif

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


	// Init BUZZER. ----------------------------------------------------------

	broadcast_message("Waiting for IMU\n\r");
	__delay32(_200MILLISEC);  // Wait for the IMU to boot

	while (INT_IMU==0) {}		// Wait for the IMU to boot
	read_imu_version(imu_board_id); imu_board_id[8] = 0;
	sprintf(message,"IMU Is Ready: %s\n\r%c[2J",imu_board_id,27);
	broadcast_message(message);

	unsigned int bigcounter = 0;
	unsigned int counter = 0;
	LED_RED = 0;
	while (1) {
		LED_ORNG = 1 - LED_ORNG;
		counter = 0;
		while(counter < 80)	
		{
			__delay32(5*_10MILLISEC);
#if 0
			switch (bigcounter % 6) {
				case 0:
					updateIMUData(IMU_UPDATE_EULER);
					sprintf(message, "ER %.2f %.2f %.2f\n",
							imuData.euler[0],imuData.euler[1],imuData.euler[2]);
					break;
				case 1:
					updateIMUData(IMU_UPDATE_ACCEL);
					sprintf(message, "AC %.2f %.2f %.2f\n",
							imuData.accel[0],imuData.accel[1],imuData.accel[2]);
					break;
				case 2:
					updateIMUData(IMU_UPDATE_GYRO);
					sprintf(message, "GR %.2f %.2f %.2f\n",
							imuData.gyro[0],imuData.gyro[1],imuData.gyro[2]);
					break;
				case 3:
					updateIMUData(IMU_UPDATE_MAG);
					sprintf(message, "MG %.2f %.2f %.2f\n",
							imuData.mag[0],imuData.mag[1],imuData.mag[2]);
					break;
				case 4:
					updateIMUData(IMU_UPDATE_PRES|IMU_UPDATE_ALT);
					sprintf(message, "AT %.2f %.2f\n",
							imuData.pressure,imuData.dist_bottom);
					break;
				case 5:
					updateIMUData(IMU_UPDATE_ALT_GYROZ_EULERZ);
					sprintf(message, "CL %.2f %.2f %.2f\n",
							imuData.dist_bottom,imuData.gyro[2],imuData.euler[2]);
					break;
				default:
					break;
			}
#else
			updateIMUData(IMU_UPDATE_ALL | IMU_UPDATE_QUATERNION | IMU_UPDATE_GYRO_BIAS);
			sprintf(message, "%c[2J%c[;HBoard %s\nER %+.2f %+.2f %+.2f\nEF %+.2f %+.2f %+.2f\nAC %+.2f %+.2f %+.2f\nGR %+.2f %+.2f %+.2f\nMG %+.2f %+.2f %+.2f\nPR %+.2f AL %+.2f %+.2f %+.2f\nCL %+.2f %+.2f %+.2f\n",27,27,imu_board_id,
					imuData.euler[0],imuData.euler[1],imuData.euler[2],
					imuData.euler_filt[0],imuData.euler_filt[1],imuData.euler_filt[2],
					imuData.accel[0],imuData.accel[1],imuData.accel[2],
					imuData.gyro[0],imuData.gyro[1],imuData.gyro[2],
					imuData.mag[0],imuData.mag[1],imuData.mag[2],
					imuData.pressure,imuData.altitude,imuData.abs_altitude,imuData.ground_altitude,
					imuData.dist_bottom,imuData.gyro[2],imuData.abs_altitude);
#if 1
			broadcast_message(message);
			sprintf(message, "BS %+.2f %+.2f %+.2f\nQT %+.3e %+.3e %+.3e %+.3e\n",
					imuData.gyro_bias[0], imuData.gyro_bias[1], imuData.gyro_bias[2],
					imuData.quaternion[0], imuData.quaternion[1], imuData.quaternion[2], imuData.quaternion[3]);
#endif
#endif

			broadcast_message(message);
			counter += 1;

		}	
		bigcounter += 1;
	}
	
}	// End of main


//====================================================
//====    START OF INTERRUPT SERVICE ROUTINES    =====
//====================================================

// TIInterrupt is declared in the coax_rc library

// T4Interrupt is declared in the control library


#if 1
//=============================================================================
//Error traps
//=============================================================================

//====================================
//Oscillator Fail Error trap routine
//====================================

void _ISR _OscillatorFail(void)
{
	INTCON1bits.OSCFAIL=0;		// fail flag clearing
	__asm__ volatile ("reset");
}

	//====================================
	//Address Error trap routine
	//====================================

void _ISR _AddressError(void) 	
{	
	INTCON1bits.ADDRERR=0;		// fail flag clearing
	__asm__ volatile ("reset");
}

//====================================
//Stack Error trap routine
//====================================

void _ISR _StackError(void)		
{		
	INTCON1bits.STKERR=0;		// fail flag clearing
	__asm__ volatile ("reset");
}

//====================================
//Math (Arithmetic) Error trap routine
//====================================

void _ISR _MathError(void)		
{	
	INTCON1bits.MATHERR=0;		// fail flag clearing
	__asm__ volatile ("reset");
}
#endif

