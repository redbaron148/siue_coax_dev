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
#include "com/sbchannel_uart1.h"

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


void sbCoaxUARTInit()
{
	static int first_run = 1;
	if (first_run) {
		e_activate_agenda(timeout_increment,0);
		e_init_uart1();
		e_init_uart2();
	}
	first_run = 0;
}


static
int sbcoaxuart_open(void * _data) 
{
	return 0;
}

static
int sbcoaxuart_close(void * _data) 
{
	return 0;
}

static
int sbcoaxuart_destroy(void * _data) 
{
	return 0;
}

static
int sbcoaxuart1_send(void * _data,const unsigned char *msg, size_t size) 
{
	e_send_uart1_char((const char*)msg,size);
	return 0;
}

static
int sbcoaxuart1_receive(void * _data, unsigned char *msg, size_t size) 
{
	unsigned int count = 0 ;
	while (e_getchar_uart1((char*)msg)) {
		msg++;
		count += 1;
	}
	return count;
}

static
int sbcoaxuart1_sendall(void * _data, const unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	if (timeout_ms==0) {
		e_send_uart1_char((const char*)msg,size);
		while (e_uart1_sending()) ;
		return 0;
	} else {
		timeout_counter=0;
		e_set_agenda_cycle(timeout_increment,10);
		e_send_uart1_char((const char*)msg,size);
		while (e_uart1_sending()) {
			if (timeout_counter > timeout_ms) {
				e_set_agenda_cycle(timeout_increment,0);
				return -1;
			}
		}
		e_set_agenda_cycle(timeout_increment,0);
	}
	return 0;
}

static
int sbcoaxuart1_waitbuffer(void * _data,  unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	unsigned int readbytes;
	readbytes = 0;
	if (timeout_ms==0) {
		while (readbytes < size) {
			if (e_getchar_uart1((char*)msg)) {
				msg ++;
				readbytes += 1;
			}
		}
	} else {
		timeout_counter = 0;
		e_set_agenda_cycle(timeout_increment,10);
		while (readbytes < size)
		{
			if (e_getchar_uart1((char*)msg)) {
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
}

static
int sbcoaxuart1_waitdata(void * _data, unsigned int timeout_ms) 
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
int sbcoaxuart1_flush(void * _data) 
{
	char dump;
	while (e_getchar_uart1(&dump));
	return 0;
}

static
int sbcoaxuart2_send(void * _data,const unsigned char *msg, size_t size) 
{
	e_send_uart2_char((const char*)msg,size);
	return 0;
}

static
int sbcoaxuart2_receive(void * _data, unsigned char *msg, size_t size) 
{
	unsigned int count = 0 ;
	while (e_getchar_uart2((char*)msg)) {
		msg++;
		count += 1;
	}
	return count;
}

static
int sbcoaxuart2_sendall(void * _data, const unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	if (timeout_ms==0) {
		e_send_uart2_char((const char*)msg,size);
		while (e_uart2_sending()) ;
		return 0;
	} else {
		timeout_counter=0;
		e_set_agenda_cycle(timeout_increment,10);
		e_send_uart2_char((const char*)msg,size);
		while (e_uart2_sending()) {
			if (timeout_counter > timeout_ms) {
				e_set_agenda_cycle(timeout_increment,0);
				return -1;
			}
		}
		e_set_agenda_cycle(timeout_increment,0);
	}
	return 0;
}

static
int sbcoaxuart2_waitbuffer(void * _data,  unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	unsigned int readbytes;
	readbytes = 0;
	if (timeout_ms==0) {
		while (readbytes < size) {
			if (e_getchar_uart2((char*)msg)) {
				msg ++;
				readbytes += 1;
			}
		}
	} else {
		timeout_counter = 0;
		e_set_agenda_cycle(timeout_increment,10);
		while (readbytes < size)
		{
			if (e_getchar_uart2((char*)msg)) {
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
}

static
int sbcoaxuart2_waitdata(void * _data, unsigned int timeout_ms) 
{
	if (timeout_ms == 0) {
		while (!e_ischar_uart2());
		return 0;
	} else {
		timeout_counter = 0;
		e_set_agenda_cycle(timeout_increment,10);
		while (timeout_counter < timeout_ms)
		{
			if (e_ischar_uart2()) {
				e_set_agenda_cycle(timeout_increment,0);
				return 0;
			}
		}
		e_set_agenda_cycle(timeout_increment,0);
	}
	return -1;
}

static
int sbcoaxuart2_flush(void * _data) 
{
	char dump;
	while (e_getchar_uart2(&dump));
	return 0;
}


static 
struct SBChannelFunction functions1 = {
	.destroy = sbcoaxuart_destroy,
	.open = sbcoaxuart_open,
	.close = sbcoaxuart_close,

	.send = sbcoaxuart1_send,
	.receive = sbcoaxuart1_receive,

	.sendall = sbcoaxuart1_sendall,
	.waitbuffer = sbcoaxuart1_waitbuffer,
	.waitdata = sbcoaxuart1_waitdata,

	.flush = sbcoaxuart1_flush
};

struct SBChannelFunction* SB_COAX_UART1_PIC30_FUNCTIONS = &functions1;


static 
struct SBChannelFunction functions2 = {
	.destroy = sbcoaxuart_destroy,
	.open = sbcoaxuart_open,
	.close = sbcoaxuart_close,

	.send = sbcoaxuart2_send,
	.receive = sbcoaxuart2_receive,

	.sendall = sbcoaxuart2_sendall,
	.waitbuffer = sbcoaxuart2_waitbuffer,
	.waitdata = sbcoaxuart2_waitdata,

	.flush = sbcoaxuart2_flush
};

struct SBChannelFunction* SB_COAX_UART2_PIC30_FUNCTIONS = &functions2;


