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
#include <stdlib.h>
#include <stdio.h>
#include <Winsock2.h>
#include <ws2bth.h>
#include <strsafe.h>
#include <windows.h>
#include <time.h>


#include "com/sbchannel_btwin.h"




struct SBBTWinChannelData {
	int outputErrors;
	HANDLE hdl;
	char * port;
	unsigned int speed;
};

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

struct SBBTWinChannelData *sbBTWinAlloc(const char *port, unsigned int speed)
{
	struct SBBTWinChannelData *data = NULL;
	data = (struct SBBTWinChannelData*)calloc(1,sizeof(struct SBBTWinChannelData));
	if (!data) return NULL;

	data->port = _strdup(port);
	data->hdl = INVALID_HANDLE_VALUE;
	data->outputErrors = 1;
	data->speed = speed;

	return data;
}



static
int sbbtwin_open(void * _data) 
{
	DCB dcb;
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
	printf("Opening serial port: %s\n",that->port);
	that->hdl = CreateFile( that->port,  
			GENERIC_READ | GENERIC_WRITE, 
			0, 0, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, 0);
	if (that->hdl == INVALID_HANDLE_VALUE) {
		// error opening port; abort
		if (that->outputErrors) {
			printf("-FATAL- | Unable to open serial port %s (%d)\n",that->port,GetLastError());
		}
		return -1;
	}


	SecureZeroMemory(&dcb, sizeof(DCB));
	dcb.DCBlength = sizeof(DCB);
	if (!GetCommState(that->hdl, &dcb)) {     // get current DCB
      // Error in GetCommState
		if (that->outputErrors) {
			printf("-WARNING- | Unable to get serial port %s/%d properties (%d)\n",that->port,that->hdl,GetLastError());
		}
      return 0;
	}

	dcb.BaudRate = that->speed;

	//// Set new state.
	if (!SetCommState(that->hdl, &dcb)) {
		
		if (that->outputErrors) {
			printf("-WARNING- | Unable to set serial port %s/%d properties (%d)\n",that->port,that->hdl,GetLastError());
		}
		return 0;
	}

	return 0;
}

static
int sbbtwin_close(void * _data) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;

	if ( INVALID_HANDLE_VALUE != that->hdl ) {
		CloseHandle(that->hdl);
		that->hdl = INVALID_HANDLE_VALUE;
	}
	return 0;
}

static
int sbbtwin_destroy(void * _data) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
	if (that) {
		free(that->port);
		if (that->hdl != INVALID_HANDLE_VALUE) {
			sbbtwin_close(that);
		}
		free(that);
	}

	return 0;
}


	static
int sbbtwin_send(void * _data,const unsigned char *msg, size_t size) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
	COMMTIMEOUTS timeouts;
	DWORD written;
	BOOL fRes = FALSE;
	OVERLAPPED osWrite = {0};

	timeouts.ReadIntervalTimeout = MAXDWORD; 
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 5;

	if (!SetCommTimeouts(that->hdl, &timeouts)){
		// Error setting time-outs.
		return -1;
	}

	// Create this writes OVERLAPPED structure hEvent.
	osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osWrite.hEvent == NULL) {
		// Error creating overlapped event handle.
		return -1;
	}

	// Issue write.
	if (!WriteFile(that->hdl, msg, size, &written, &osWrite)) {
		if (GetLastError() != ERROR_IO_PENDING) { 
			// WriteFile failed, but it isn't delayed. Report error and abort.
			fRes = FALSE;
		} else if (!GetOverlappedResult(that->hdl, &osWrite, &written, TRUE)) {
			// Write is pending.
			fRes = FALSE;
		} else {
			// Write operation completed successfully.
			fRes = TRUE;
		}

	} else {
		// WriteFile completed immediately.
		fRes = TRUE;
	}

	CloseHandle(osWrite.hEvent);

	if (!fRes  && that->outputErrors) {
		printf("send() call failed with error '%d'\n",GetLastError());
	}

	return written;
}

static
int sbbtwin_receive(void * _data, unsigned char *msg, size_t size) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
	COMMTIMEOUTS timeouts;
	DWORD bread;
	BOOL fRes = FALSE;
	OVERLAPPED osRead = {0};

	timeouts.ReadIntervalTimeout = MAXDWORD; 
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;

	if (!SetCommTimeouts(that->hdl, &timeouts)){
		// Error setting time-outs.
		return -1;
	}
	
	// Create this writes OVERLAPPED structure hEvent.
	osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osRead.hEvent == NULL) {
		// Error creating overlapped event handle.
		return -1;
	}

	// Issue write.
	if (!ReadFile(that->hdl, msg, size, &bread, &osRead)) {
		if (GetLastError() != ERROR_IO_PENDING) { 
			// WriteFile failed, but it isn't delayed. Report error and abort.
			fRes = FALSE;
		} else if (!GetOverlappedResult(that->hdl, &osRead, &bread, TRUE)) {
			// Write is pending.
			fRes = FALSE;
		} else {
			// Write operation completed successfully.
			fRes = TRUE;
		}

	} else {
		// WriteFile completed immediately.
		fRes = TRUE;
	}

	CloseHandle(osRead.hEvent);


	if (!fRes  && that->outputErrors) {
		printf("receive() call failed with error '%d'\n",GetLastError());
	}

	return bread;
}


static
int sbbtwin_sendall(void * _data, const unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
	COMMTIMEOUTS timeouts;
	DWORD written;
	BOOL fRes;
	OVERLAPPED osWrite = {0};

	timeouts.ReadIntervalTimeout = MAXDWORD; 
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = 0;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = timeout_ms;

	if (!SetCommTimeouts(that->hdl, &timeouts)){
		// Error setting time-outs.
		return -1;
	}


	// Create this writes OVERLAPPED structure hEvent.
	osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osWrite.hEvent == NULL) {
		// Error creating overlapped event handle.
		return -1;
	}

	// Issue write.
	if (!WriteFile(that->hdl, msg, size, &written, &osWrite)) {
		if (GetLastError() != ERROR_IO_PENDING) { 
			// WriteFile failed, but it isn't delayed. Report error and abort.
			fRes = FALSE;
		} else {
			// Write is pending.
			DWORD dwRes = WaitForSingleObject(osWrite.hEvent, timeout_ms+5);
			switch(dwRes)
			{
				// OVERLAPPED structure's event has been signaled. 
				case WAIT_OBJECT_0:
					if (!GetOverlappedResult(that->hdl, &osWrite, &written, FALSE)) {
						fRes = FALSE;
					} else {
						// Write operation completed successfully.
						fRes = TRUE;
					}
					break;

				default:
					// An error has occurred in WaitForSingleObject.
					// This usually indicates a problem with the
					// OVERLAPPED structure's event handle.
					fRes = FALSE;
					break;
			}
		}
	} else {
		// WriteFile completed immediately.
		fRes = TRUE;
	}
	CloseHandle(osWrite.hEvent);
	if (!fRes  && that->outputErrors) {
		printf("send() call failed with error '%d'\n",GetLastError());
	}
	return (written == size)?0:-1;
}

static
int sbbtwin_waitbuffer(void * _data,  unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
	COMMTIMEOUTS timeouts;
	DWORD bread;
	BOOL fRes = FALSE;
	OVERLAPPED osRead = {0};

	timeouts.ReadIntervalTimeout = 0; 
	timeouts.ReadTotalTimeoutMultiplier = 0;
	timeouts.ReadTotalTimeoutConstant = timeout_ms;
	timeouts.WriteTotalTimeoutMultiplier = 0;
	timeouts.WriteTotalTimeoutConstant = 0;

	if (!SetCommTimeouts(that->hdl, &timeouts)){
		// Error setting time-outs.
		return -1;
	}

	
	// Create this writes OVERLAPPED structure hEvent.
	osRead.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osRead.hEvent == NULL) {
		// Error creating overlapped event handle.
		return -1;
	}

	// Issue read.
	if (!ReadFile(that->hdl, msg, size, &bread, &osRead)) {
		if (GetLastError() != ERROR_IO_PENDING) { 
			// WriteFile failed, but it isn't delayed. Report error and abort.
			fRes = FALSE;
		} else {
			// Write is pending.
			DWORD dwRes = WaitForSingleObject(osRead.hEvent, timeout_ms+5);
			switch(dwRes)
			{
				// OVERLAPPED structure's event has been signaled. 
				case WAIT_OBJECT_0:
					if (!GetOverlappedResult(that->hdl, &osRead, &bread, FALSE)) {
						fRes = FALSE;
					} else {
						// Write operation completed successfully.
						fRes = TRUE;
					}
					break;

				default:
					// An error has occurred in WaitForSingleObject.
					// This usually indicates a problem with the
					// OVERLAPPED structure's event handle.
					fRes = FALSE;
					break;
			}
		}
	} else {
		// WriteFile completed immediately.
		fRes = TRUE;
	}

	CloseHandle(osRead.hEvent);


	if (!fRes  && that->outputErrors) {
		printf("receive() call failed with error '%d'\n",GetLastError());
	}


	return (bread == size)?0:-1;
}

static
int sbbtwin_waitdata(void * _data, unsigned int timeout_ms) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
	OVERLAPPED osStatus = {0}; // dont need to destroy, we are in C++
	DWORD      dwRes, dwOvRes;
	DWORD      dwCommEvent;
	DWORD      dwStoredFlags;

	dwStoredFlags = EV_RXCHAR;
	if (!SetCommMask(that->hdl, dwStoredFlags)) {
		// error setting communications mask
		return -1;
	}

	osStatus.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osStatus.hEvent == NULL) {
		// error creating event; abort
		return -1;
	}

	// Issue a status event check if one hasn't been issued already.
	if (!WaitCommEvent(that->hdl, &dwCommEvent, &osStatus)) {
		if (GetLastError() != ERROR_IO_PENDING) {
			// error in WaitCommEvent; abort
			goto cleanup_and_fail;
		}
	} else {
		CloseHandle(osStatus.hEvent);
		osStatus.hEvent = NULL;
		return (dwCommEvent & dwStoredFlags)?0:1; 
	}

	// Check on overlapped operation.
	// Wait a little while for an event to occur.
	dwRes = WaitForSingleObject(osStatus.hEvent, timeout_ms);
	switch(dwRes)
	{
		// Event occurred.
		case WAIT_OBJECT_0: 
			if (!GetOverlappedResult(that->hdl, &osStatus, &dwOvRes, FALSE)) {
				// An error occurred in the overlapped operation;
				// call GetLastError to find out what it was
				// and abort if it is fatal.
				break;
			} else {
				// Status event is stored in the event flag
				// specified in the original WaitCommEvent call.
				// Deal with the status event as appropriate.
				CloseHandle(osStatus.hEvent);
				osStatus.hEvent = NULL;
				return (dwCommEvent & dwStoredFlags)?0:1; 
			}

		case WAIT_TIMEOUT:
		default:
			break;
	}
cleanup_and_fail:
	CloseHandle(osStatus.hEvent);
	osStatus.hEvent = NULL;
	return -1;
}

static
int sbbtwin_flush(void * _data) 
{
#if 1
	// It is not clear if that does what we expect. But we should not use it
	// anyway...
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
	PurgeComm(that->hdl,
			PURGE_RXABORT|PURGE_RXCLEAR|PURGE_TXABORT|PURGE_TXCLEAR);
#else
	unsigned char trash[32];
	while (sbbtwin_receive(_data,trash,32)>0);
#endif
	return 0;
}

#ifdef WIN32

// Visual studio does not support named initialization (C99 standard...)

struct SBChannelFunction functions = {
	sbbtwin_destroy,
	sbbtwin_open,
	sbbtwin_close,

	sbbtwin_send,
	sbbtwin_receive,

	sbbtwin_sendall,
	sbbtwin_waitbuffer,
	sbbtwin_waitdata,

	sbbtwin_flush
};
#else
static 
struct SBChannelFunction functions = {
	.destroy = sbbtwin_destroy,
	.open = sbbtwin_open,
	.close = sbbtwin_close,

	.send = sbbtwin_send,
	.receive = sbbtwin_receive,

	.sendall = sbbtwin_sendall,
	.waitbuffer = sbbtwin_waitbuffer,
	.waitdata = sbbtwin_waitdata,

	.flush = sbbtwin_flush
};
#endif

struct SBChannelFunction* SB_BTWIN_FUNCTIONS = &functions;

