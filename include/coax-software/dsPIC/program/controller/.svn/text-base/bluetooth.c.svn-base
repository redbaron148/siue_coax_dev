/********************************************************************************

			Bluetooth for e-puck
			Version 1.0 July 2006 Michael Bonani


This file is part of the e-puck library license.
See http://www.e-puck.org/index.php?option=com_content&task=view&id=18&Itemid=45

(c) 2006-2007 Michael Bonani

Robotics system laboratory http://lsro.epfl.ch
Laboratory of intelligent systems http://lis.epfl.ch
Swarm intelligent systems group http://swis.epfl.ch
EPFL Ecole polytechnique federale de Lausanne http://www.epfl.ch

Adapted for Coax by Gilles Caprari, october 2008
Cut-down by Cedric Pradalier, January 2009

**********************************************************************************/

/*! \file
 * \ingroup bluetooth
 * \brief Manage Bluetooth.
 *
 * This module manage the Bluetooth device.
 * \author Code: Michael Bonani \n Doc: Jonathan Besuchet
 */

//#include <configs/init_port.h>
#include <uart/coax_uart.h>
#include <configs/coax_ports.h>
#include <configs/coax_config.h>

#include "bluetooth.h"

/*! \brief Write the PIN number on this e-puck's bluetooth module
 * \param PIN A pointer to store the PIN number
 * \return bluetooth error if one occur, 0 otherwise
 */
char e_bt_write_local_pin_number(const char PIN[4])
{
	char send[30];
	char read[10];
	unsigned long timeout, timeoutms;
	int i;
	char c;
	int numberlength;

	numberlength=4;
	//send_uart2(PIN,numberlength);
	send[0]=0x02; 
	send[1]=0x52;
	send[2]=0x17;
	send[3]=numberlength+1;
	send[4]=0x00;
	send[5]=(send[1]+send[2]+send[3]);
	send[6]=numberlength;
	for (i=0;i<numberlength;i++)
		send[i+7]=PIN[i];
	send[7+numberlength]=0x03;
	e_send_uart2_char(send,numberlength+8);

	i=0;
	c=0;

	timeout = timeoutms = 0;
	do {
		if (e_getchar_uart2(&read[i]))		//read response
		{	
			c=read[i];
			i++;
		}	
		timeoutms ++;
		if (timeoutms >= MILLISEC) {
			timeoutms = 0;
			timeout ++;
		}
		if (timeout > 100) {
			return -1;
		}
	}
	while (((char)c != 0x03)||(i<(read[3]+6)));
	read[i]='\0';
	return read[6];//return error 0=no error
}

/*! \brief Write the name on this e-puck's bluetooth module
 * \param name A pointer to store the name
 * \return bluetooth error if one occur, 0 otherwise
 */
char e_bt_write_local_name(const char name[16])
{
	char send[40];
	char read[10];
	int i;
	char c;
	unsigned long timeout, timeoutms;
	int namelength;

	namelength=16;
	namelength++;//add null caracter
	//send_uart2(PIN,numberlength);
	send[0]=0x02; //send PIN request
	send[1]=0x52;
	send[2]=0x04;
	send[3]=namelength+1;
	send[4]=0x00;
	send[5]=(send[1]+send[2]+send[3]);
	send[6]=namelength;
	for (i=0;i<namelength;i++)
		send[i+7]=name[i];
	send[7+namelength]=0x03;
	e_send_uart2_char(send,namelength+8);

	i=0;
	c=0;

	timeout = timeoutms = 0;
	do {
		if (e_getchar_uart2(&read[i]))		//read response
		{	
			c=read[i];
			i++;
		}	
		timeoutms ++;
		if (timeoutms >= MILLISEC) {
			timeoutms = 0;
			timeout ++;
		}
		if (timeout > 100) {
			return -1;
		}

	} while (((char)c != 0x03)||(i<(read[3]+6)));
	read[i]='\0';
	return read[6];//return error 0=no error
}

