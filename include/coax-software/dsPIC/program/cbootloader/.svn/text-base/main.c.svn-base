/**************************************
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
 **************************************/

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
// #define DEBUG2

#define DEBUG_CHANNEL 1
// Use a mask with 1 and 2
// #define ACTIVE_CHANNEL 1  // For ASL IMU
#define ACTIVE_CHANNEL 2  // For Coax


#if DEBUG_CHANNEL == 0
#define DEBUG(s) broadcast_string((char*)s)
#elif DEBUG_CHANNEL == 1
#define DEBUG(s) send_string_uart1((unsigned char*)s)
#elif DEBUG_CHANNEL == 2
#define DEBUG(s) send_string_uart2((unsigned char*)s)
#endif

/* ===== STANDARD INCLUDES. ============================================= */
#include <p33FJ256GP506.h>
#include <libpic30.h>


#include <uart.h>


/* ===== CUSTOM MADE INCLUDES. ========================================== */
#include "utils.h"


void InitOscillator(void)
{

	// Configure Oscillator to operate the device at 40Mhz
	// Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
	// Fosc= 25M*40/(2*2)=80Mhz for 25M input clock

	CLKDIVbits.PLLPRE=3;		// N1=5
	CLKDIVbits.PLLPOST=0;		// N2=2
	PLLFBD=30;					// M=32
	OSCTUN=0;					// Tune FRC oscillator, if FRC is used

	// Disable Watch Dog Timer
	RCONbits.SWDTEN=0;

	// clock switching to incorporate PLL
	__builtin_write_OSCCONH(0x03);		// Initiate Clock Switch to Primary
	// Oscillator with PLL (NOSC=0b011)
	__builtin_write_OSCCONL(0x01);		// Start clock switching
	while (OSCCONbits.COSC != 0b011);	// Wait for Clock switch to occur

	// Wait for PLL to lock
	while(OSCCONbits.LOCK!=1) {};

}


void send_string_uart2(unsigned char *txt) 
{
	while(U2STAbits.UTXBF); /* wait if the buffer is full */
	while (*txt) {
		U2TXREG = *txt++;
		while(U2STAbits.UTXBF); /* wait if the buffer is full */
	}
}

void send_char_uart2(unsigned char txt) 
{
	while(U2STAbits.UTXBF); /* wait if the buffer is full */
	U2TXREG = txt;
	while(U2STAbits.UTXBF); /* wait if the buffer is full */
}

int get_char_uart2(unsigned char *txt)
{
	if (U2STAbits.URXDA) {
		*txt = U2RXREG;
		return 1;
	}
	return 0;
}

void send_string_uart1(unsigned char *txt) 
{
	while(U1STAbits.UTXBF); /* wait if the buffer is full */
	while (*txt) {
		U1TXREG = *txt++;
		while(U1STAbits.UTXBF); /* wait if the buffer is full */
	}
}

void send_char_uart1(unsigned char txt) 
{
	while(U1STAbits.UTXBF); /* wait if the buffer is full */
	U1TXREG = txt;
	while(U1STAbits.UTXBF); /* wait if the buffer is full */
}

int get_char_uart1(unsigned char *txt)
{
	if (U1STAbits.URXDA) {
		*txt = U1RXREG;
		return 1;
	}
	return 0;
}

void send_char_uart(int id, unsigned char txt) {
	switch (id) {
		case 1: send_char_uart1(txt);break;
		case 2: send_char_uart2(txt);break;
		default: break;
	}
}

int get_char_uart(int id, unsigned char *txt) {
	switch (id) {
		case 1: return get_char_uart1(txt);
		case 2: return get_char_uart2(txt);
		default: return 0;
	}
}

void broadcast_string(char * text) {
	send_string_uart1((unsigned char*)text);
	send_string_uart2((unsigned char*)text);
}


/*********************************************************************
 * Function Name     : OpenUART1                                      *
 * Description       : This function configures the UART mode,        * 
 *                     UART Interrupt modes and the Baud Rate         *
 * Parameters        : unsigned int config1 operation setting         *
 *                     unsigned int config2 TX & RX interrupt modes   *
 *                     unsigned int ubrg baud rate setting            *
 * Return Value      : None                                           *
 *********************************************************************/

void OpenUART1(unsigned int config1,unsigned int config2, unsigned int ubrg)
{
	U1BRG  = ubrg;     /* baud rate */
	U1MODE = config1;  /* operation settings */
	U1STA = config2;   /* TX & RX interrupt modes */
}

/*********************************************************************
 * Function Name     : OpenUART2                                      *
 * Description       : This function configures the UART mode,        *
 *                     UART Interrupt modes and the Baud Rate         *
 * Parameters        : unsigned int config1 operation setting         *
 *                     unsigned int config2 TX & RX interrupt modes   *
 *                     unsigned int ubrg baud rate setting            *
 * Return Value      : None                                           *      
 *********************************************************************/

void OpenUART2(unsigned int config1,unsigned int config2, unsigned int ubrg)
{
	U2BRG  = ubrg;       /* baud rate */
	U2MODE = config1;    /* operation settings */
	U2STA = config2;     /* TX & RX interrupt modes */
}

// old function. we leave it here to compatibility to old programs
void init_UART(void) 
{

	//========================================================
	//=== Set baudrate according to equation 19-2 of datasheet
	//========================================================

	unsigned int baudvalue1=87;			// 115200 bps @ 40 MIPS
	unsigned int baudvalue2=87;		// 

	unsigned int U1MODEvalue;
	unsigned int U1STAvalue;
	unsigned int U2MODEvalue;
	unsigned int U2STAvalue;

	U1MODEvalue = 	UART_EN &						// UART Module Enabled 
		UART_IDLE_CON &					// Work in IDLE mode
		UART_IrDA_DISABLE &				// IrDA encoder and decoder disabled
		UART_MODE_SIMPLEX &				// UxRTS pin in Simplex mode
		UART_UEN_00 &					// UxTX and UxRX pins are enabled and used; UxCTS and UxRTS/BCLK pins controlled by port latches
		UART_DIS_WAKE &					// Disable Wake-up on START bit Detect during SLEEP Mode bit
		UART_DIS_LOOPBACK &				// Loop back disabled
		UART_DIS_ABAUD &				// Baud rate measurement disabled or completed
		UART_UXRX_IDLE_ONE &			// UxRX Idle state is zero
		UART_BRGH_FOUR &				// BRG generates 4 clocks per bit period
		0xFFF9 &						// Instead of UART_NO_PAR_8BIT that is WRONG in uart.h
		UART_1STOPBIT;					// 1 stop bit

	//IPC2bits.U1RXIP = 2;							// Uart int priority
	//IPC3bits.U1TXIP = 2;
	U1MODEvalue = 0b1100100000001000;				// 
	U1STAvalue  = 0b1000010100000000;

	/* Configure uart2 receive and transmit interrupt */
	//ConfigIntUART2(	UART_RX_INT_EN &
	//				UART_RX_INT_PR1 &
	//				UART_TX_INT_EN &
	//				UART_TX_INT_PR3);

	//IPC7bits.U2RXIP = 2;							// Uart int priority
	//IPC7bits.U2TXIP = 2;
	U2MODEvalue = 0b1100100000001000;
	U2STAvalue  = 0b1000010100000000;

	/* Open UARTx */
	OpenUART1(U1MODEvalue, U1STAvalue, baudvalue1);
	OpenUART2(U2MODEvalue, U2STAvalue, baudvalue2);

}




/* ===== CONFIGURATION BITS ============================================= */
_FOSCSEL(FNOSC_PRI);								// Primary (XT, HS, EC) Oscillator
_FOSC(FCKSM_CSECME & OSCIOFNC_OFF  & POSCMD_HS);  
// Clock Switching is enabled and Fail Safe Clock Monitor is disabled
// OSC2 Pin Function: OSC2 is Clock Output
// Primary Oscillator Mode: XT Crystanl
//_FWDT(FWDTEN_OFF);              					// Watchdog Timer Enabled/disabled by user software
_FWDT(FWDTEN_OFF & WINDIS_OFF)						// Wdt
	_FPOR(FPWRT_PWR128);								// POR (Power On Reset)

#define IdTypePIC 0x71


#define GETCHAR(x) get_char_uart(active_uart,x)
#define SENDCHAR(x) send_char_uart(active_uart,x | 0x80)
#define InterruptOFF() {__asm__ volatile ("disi	#10000");}
#define InterruptON() {__asm__ volatile ("disi	#2");}
#define MICROSEC   		40uL     		//  # of cycles for 1uSec delay (minumum is 11 cycles)
#define MILLISEC   		(1000*MICROSEC)	// # of cycles for 1mSec delay (minumum is 11 cycles)

void WaitMiliSec(unsigned int mils)		// TO REMOVE !!!
{
	while (mils--) {
		__delay32(MILLISEC);
	}
}

/* ===== MAIN. ========================================================== */
extern void write_packet(unsigned char*buffer);
extern void erase_memory(unsigned char*buffer);
unsigned char packet[4+3*64+1];

int main(void) {
	unsigned int timeout,timeoutms;
	int active_uart = 0;

	// Init Oscillator. ----------------------------------------------------
	InitOscillator();	// Initialize the PLL (also disables wdt)
	WaitMiliSec(50);

	// Init UARTS. ---------------------------------------------------------	
	init_UART();

	// Welcome Message-----------------------------------------------------

	DEBUG("\n\n\rWELCOME to CoaX Bootloader \n");	// Welcome msg
#if DEBUG_CHANNEL != 1
	DEBUG("Listening on UART: 1\n");
#endif
#if DEBUG_CHANNEL != 2
	DEBUG("Listening on UART: 2\n");
#endif

	timeout = 0;
	while (1) {
		InterruptOFF();
#if DEBUG_CHANNEL != 1
		if (ACTIVE_CHANNEL & 1) {
			if (get_char_uart1(packet)) {
				active_uart = 1;
				DEBUG("Active UART: 1\n");
				break;
			}
		}
#endif
#if DEBUG_CHANNEL != 2
		if (ACTIVE_CHANNEL & 2) {
			if (get_char_uart2(packet)) {
				DEBUG("Active UART: 2\n");
				active_uart = 2;
				break;
			}
		}
#endif
		// Wait a ms
		WaitMiliSec(1);
		timeout ++;
		if (timeout >= 2000) {
			goto cleanup_and_leave;
		}
	}

	if (packet[0] != 0xC1) {
		DEBUG("Invalid first token\n");	// Welcome msg
		goto cleanup_and_leave;
	}

	InterruptOFF();
	SENDCHAR(IdTypePIC);
	DEBUG("Received starting token\n");	// Welcome msg

	while(1)
	{	
		register unsigned int i,sum=0, packet_count = 0;
		InterruptOFF();
		SENDCHAR('K');
#ifdef DEBUG2
		DEBUG("Waiting packet\n");	// Welcome msg
#endif
		timeoutms = 0;
		for (i=0;i<4;i++) {
			timeout = 0;
			while (1) {
				unsigned char buffer;
				InterruptOFF();
				if ( GETCHAR(&buffer) ){
					packet[packet_count++] = buffer;
					sum += buffer;
					break;
				} else {
					timeoutms ++;
					if (timeoutms >= MILLISEC) {
						timeoutms = 0;
						timeout ++;
					}
					if (timeout >= 2000) {
						DEBUG("Timeout\n");	// Welcome msg
						goto cleanup_and_leave;
					}
				}
			}
		}


		if (packet[3]==0xFF) {
			WaitMiliSec(packet[2]);
			break;
		}
#ifdef DEBUG2
		DEBUG("Waiting body\n");	// Welcome msg
#endif
		timeoutms = 0;
		for (i=0;i<3*64+1;i++) {
			InterruptOFF();
			timeout = 0;
			while (1) {
				unsigned char buffer;
				InterruptOFF();
				if ( GETCHAR(&buffer) ){
					packet[packet_count++] = buffer;
					sum += buffer;
					break;
				} else {
					timeoutms ++;
					if (timeoutms >= MILLISEC) {
						timeoutms = 0;
						timeout ++;
					}
					if (timeout >= 2000) {
						DEBUG("Timeout\n");	// Welcome msg
						goto cleanup_and_leave;
					}
				}
			}
		}
		sum &= 0xFF;

		if (sum) {
			InterruptOFF();
			SENDCHAR('N');
			DEBUG("Invalid Check sum\n");	// Welcome msg
			continue;
		}
		if (packet[3] == 0xFE) {		
			InterruptOFF();
#ifdef DEBUG2
			DEBUG("Erasing packet\n");	// Welcome msg
#endif
			erase_memory(packet);
#ifdef DEBUG2
			DEBUG("Done\n");	// Welcome msg
#endif
		} else {
			InterruptOFF();
#ifdef DEBUG2
			DEBUG("Writing packet\n");	// Welcome msg
#endif
			write_packet(packet);
#ifdef DEBUG2
			DEBUG("Done\n");	// Welcome msg
#endif
			DEBUG("#");	// Welcome msg
		}
	}

cleanup_and_leave:
	DEBUG("\nFinished\n");	// Welcome msg
#ifdef DEBUG1
	WaitMiliSec(100);
#endif
	__asm__ volatile ("goto 0x200");

	return 0;
}
