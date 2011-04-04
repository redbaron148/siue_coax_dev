/*************************************************************
*
* API and Communication library for the Skybotix AG
* helicopters, and particularly COAX
*
* Developed by Cedric Pradalier: cedric.pradalier@skybotix.ch
* Send modification or corrections as patches (diff -Naur)
* Copyright: Skybotix AG, 2009-2012
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
* 
*************************************************************/
#include "sbchannel_uart1.h"

//#include <e_uart_char.h>
#include <uart/coax_uart.h>
#include <agenda/e_agenda.h>


/*
	int (*destroy)(void*);

	int (*open)(void*);
	int (*close)(void*);

	int (*send)(void*,const unsigned char *data, size_t size);
	int (*receive)(void*,unsigned char *data, size_t size);

	int (*sendall)(void*,const unsigned char *data, size_t size,unsigned int timeout_ms);
	int (*waitbuffer)(void*,unsigned char *data, size_t size,unsigned int timeout_ms);
	int (*waitdata)(void*,unsigned int timeout_ms);

	int (*flush)(void*);
*/

static unsigned int timeout_counter = 0;

static void timeout_increment() 
{
	timeout_counter += 1;
}


void sbUARTInit()
{
	e_activate_agenda(timeout_increment,0);
	e_init_uart1();
}


static
int sbuart_open(void * _data) 
{
	return 0;
}

static
int sbuart_close(void * _data) 
{
	return 0;
}

static
int sbuart_destroy(void * _data) 
{
	return 0;
}

static
int sbuart_send(void * _data,const unsigned char *msg, size_t size) 
{
	e_send_uart1_char(msg,size);
	return 0;
}

static
int sbuart_receive(void * _data, unsigned char *msg, size_t size) 
{
	unsigned int count = 0 ;
	while (e_getchar_uart1(msg)) {
		msg++;
		count += 1;
	}
	return count;
}

static
int sbuart_sendall(void * _data, const unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
#if 1
	if (timeout_ms==0) {
		e_send_uart1_char(msg,size);
		while (e_uart1_sending()) ;
		return 0;
	} else {
		timeout_counter=0;
		e_set_agenda_cycle(timeout_increment,10);
		e_send_uart1_char(msg,size);
		while (e_uart1_sending()) {
			if (timeout_counter > timeout_ms) {
				e_set_agenda_cycle(timeout_increment,0);
				return -1;
			}
		}
		e_set_agenda_cycle(timeout_increment,0);
	}
	return 0;
#else
	int i;
	e_set_agenda_cycle(timeout_increment,10);
	e_set_led(4,1); e_set_led(5,0); e_set_led(6,0); e_set_led(7,0);
	for (i=0;i<size;i++) {
		e_send_uart1_char(msg,1);
		while (e_uart1_sending());
		e_set_led(4+(i&3),2);
		timeout_counter = 0;
		while (timeout_counter < 1000);
		msg++;
	}
	e_set_agenda_cycle(timeout_increment,0);
	return 0;
#endif
}

static
int sbuart_waitbuffer(void * _data,  unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
#if 1
	unsigned int readbytes;
	readbytes = 0;
	if (timeout_ms==0) {
		while (readbytes < size) {
			if (e_getchar_uart1(msg)) {
				msg ++;
				readbytes += 1;
			}
		}
	} else {
		timeout_counter = 0;
		e_set_agenda_cycle(timeout_increment,10);
		while (readbytes < size)
		{
			if (e_getchar_uart1(msg)) {
				msg ++;
				readbytes += 1;
			}
			if ((timeout_ms>0) && (timeout_counter > timeout_ms))  {
				break;
			}
		}
		e_set_agenda_cycle(timeout_increment,0);
	}
	return (readbytes == size)?0:-1;
#else
	unsigned int readbytes;
	unsigned long t0;
	timeout_counter = 0;
	e_set_agenda_cycle(timeout_increment,10);
	readbytes = 0;
	e_set_led(4,1); e_set_led(5,0); e_set_led(6,0); e_set_led(7,0);
	while (readbytes < size)
	{
		if (e_getchar_uart1(msg)) {
			msg ++;
			readbytes += 1;
			e_set_led(4+(readbytes&3),2);
			timeout_counter = 0;
			while (timeout_counter < 1000);
		}
	}
	e_set_agenda_cycle(timeout_increment,0);
	return (readbytes == size)?0:-1;
#endif
}

static
int sbuart_waitdata(void * _data, unsigned int timeout_ms) 
{
	if (timeout_ms == 0) {
		while (!e_ischar_uart1());
		return 0;
	} else {
		timeout_counter = 0;
		e_set_agenda_cycle(timeout_increment,10);
		while (timeout_counter < timeout_ms)
		{
			if (e_ischar_uart1()) {
				e_set_agenda_cycle(timeout_increment,0);
				return 0;
			}
		}
		e_set_agenda_cycle(timeout_increment,0);
	}
	return -1;
}

static
int sbuart_flush(void * _data) 
{
	char dump;
	while (e_getchar_uart1(&dump));
	return 0;
}


static 
struct SBChannelFunction functions = {
	.destroy = sbuart_destroy,
	.open = sbuart_open,
	.close = sbuart_close,

	.send = sbuart_send,
	.receive = sbuart_receive,

	.sendall = sbuart_sendall,
	.waitbuffer = sbuart_waitbuffer,
	.waitdata = sbuart_waitdata,

	.flush = sbuart_flush
};

struct SBChannelFunction* SB_UART_PIC30_FUNCTIONS = &functions;


