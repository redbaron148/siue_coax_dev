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
#include <initguid.h>
#include <time.h>


#include "sbchannel_btwin.h"




struct SBBTWinChannelData {
	int outputErrors;
	SOCKET sock;
	char * host;
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

struct SBBTWinChannelData *sbBTWinAlloc(const char *host)
{
	struct SBBTWinChannelData *data = NULL;
	data = (struct SBBTWinChannelData*)calloc(1,sizeof(struct SBBTWinChannelData));
	if (!data) return NULL;

	data->host = _strdup(host);
	data->sock = INVALID_SOCKET;
	data->outputErrors = 1;

	return data;
}

// {B62C4E8D-62CC-404b-BBBF-BF3E3BBB1374}
DEFINE_GUID(g_guidServiceClass, 0xb62c4e8d, 0x62cc, 0x404b, 0xbb, 0xbf, 0xbf, 0x3e, 0x3b, 0xbb, 0x13, 0x74);

#define CXN_BDADDR_STR_LEN                17   // 6 two-digit hex values plus 5 colons
#define CXN_SUCCESS                       0
#define CXN_ERROR                         1
#define CXN_MAX_INQUIRY_RETRY             3
#define CXN_DELAY_NEXT_INQUIRY            15

//
// TODO: use inquiry timeout SDP_DEFAULT_INQUIRY_SECONDS
//

//
// NameToBthAddr converts a bluetooth device name to a bluetooth address,
// if required by performing inquiry with remote name requests.
// This function demonstrates device inquiry, with optional LUP flags.
//
static
ULONG NameToBthAddr(IN const char * pszRemoteName, OUT BTH_ADDR * pRemoteBtAddr)
{
	INT				iResult = 0, iRetryCount = 0;
	BOOL			bContinueLookup = FALSE, bRemoteDeviceFound = FALSE;
	ULONG			ulFlags = 0, ulPQSSize = sizeof(WSAQUERYSET);
	HANDLE			hLookup = 0;
	PWSAQUERYSET	pWSAQuerySet = NULL;

	if ( ( NULL == pszRemoteName ) || ( NULL == pRemoteBtAddr ) ) {
		goto CleanupAndExit;
	}
	pWSAQuerySet = (PWSAQUERYSET) HeapAlloc(GetProcessHeap(),
			HEAP_ZERO_MEMORY,
			ulPQSSize);
	if ( NULL == pWSAQuerySet ) {
		printf("!ERROR! | Unable to allocate memory for WSAQUERYSET\n");
		goto CleanupAndExit;
	}

	//
	// Search for the device with the correct name
	//
	for ( iRetryCount = 0;
			!bRemoteDeviceFound && (iRetryCount < CXN_MAX_INQUIRY_RETRY);
			iRetryCount++ ) {
		//
		// WSALookupService is used for both service search and device inquiry
		// LUP_CONTAINERS is the flag which signals that we're doing a device inquiry.
		//
		ulFlags = LUP_CONTAINERS;

		//
		// Friendly device name (if available) will be returned in lpszServiceInstanceName
		//
		ulFlags |= LUP_RETURN_NAME;

		//
		// BTH_ADDR will be returned in lpcsaBuffer member of WSAQUERYSET
		//
		ulFlags |= LUP_RETURN_ADDR;

		if ( 0 == iRetryCount ) {
			printf("*INFO* | Inquiring device from cache...\n");
		} else {
			//
			// Flush the device cache for all inquiries, except for the first inquiry
			//
			// By setting LUP_FLUSHCACHE flag, we're asking the lookup service to do
			// a fresh lookup instead of pulling the information from device cache.
			//
			ulFlags |= LUP_FLUSHCACHE;

			//
			// Pause for some time before all the inquiries after the first inquiry
			//
			// Remote Name requests will arrive after device inquiry has
			// completed.  Without a window to receive IN_RANGE notifications,
			// we don't have a direct mechanism to determine when remote
			// name requests have completed.
			//
			printf("*INFO* | Unable to find device.  Waiting for %d seconds before re-inquiry...\n", CXN_DELAY_NEXT_INQUIRY);
			Sleep(CXN_DELAY_NEXT_INQUIRY * 1000);

			printf("*INFO* | Inquiring device ...\n");
		}

		//
		// Start the lookup service
		//
		iResult = 0;
		hLookup = 0;
		bContinueLookup = FALSE;
		ZeroMemory(pWSAQuerySet, ulPQSSize);
		pWSAQuerySet->dwNameSpace = NS_BTH;
		pWSAQuerySet->dwSize = sizeof(WSAQUERYSET);
		iResult = WSALookupServiceBegin(pWSAQuerySet, ulFlags, &hLookup);

		if ( (NO_ERROR == iResult) && (NULL != hLookup) ) {
			bContinueLookup = TRUE;
		} else if ( 0 < iRetryCount ) {
			printf("=CRITICAL= | WSALookupServiceBegin() failed with error code %d, WSALastError = %d\n", iResult, WSAGetLastError());
			goto CleanupAndExit;
		}

		while ( bContinueLookup ) {
			//
			// Get information about next bluetooth device
			//
			// Note you may pass the same WSAQUERYSET from LookupBegin
			// as long as you don't need to modify any of the pointer
			// members of the structure, etc.
			//
			// ZeroMemory(pWSAQuerySet, ulPQSSize);
			// pWSAQuerySet->dwNameSpace = NS_BTH;
			// pWSAQuerySet->dwSize = sizeof(WSAQUERYSET);
			if ( NO_ERROR == WSALookupServiceNext(hLookup,
						ulFlags,
						&ulPQSSize,
						pWSAQuerySet) ) {
				//
				// Since we're a non-unicode application, the remote
				// name in lpszServiceInstanceName will have been converted
				// from CP_UTF8 to CP_ACP, this may cause the name match
				// to fail unexpectedly.  If the app is to handle this,
				// the app needs to be unicode.
				//
				if ( ( pWSAQuerySet->lpszServiceInstanceName != NULL ) &&
						( 0==_stricmp(pWSAQuerySet->lpszServiceInstanceName, pszRemoteName) ) ) {
					//
					// Found a remote bluetooth device with matching name.
					// Get the address of the device and exit the lookup.
					//
					CopyMemory(pRemoteBtAddr,
							&((PSOCKADDR_BTH) pWSAQuerySet->lpcsaBuffer->RemoteAddr.lpSockaddr)->btAddr,
							sizeof(*pRemoteBtAddr));
					bRemoteDeviceFound = TRUE;
					bContinueLookup = FALSE;
				}
			} else {
				if ( WSA_E_NO_MORE == ( iResult = WSAGetLastError() ) ) { //No more data
					//
					// No more devices found.  Exit the lookup.
					//
					bContinueLookup = FALSE;
				} else if ( WSAEFAULT == iResult ) {
					//
					// The buffer for QUERYSET was insufficient.
					// In such case 3rd parameter "ulPQSSize" of function "WSALookupServiceNext()" receives
					// the required size.  So we can use this parameter to reallocate memory for QUERYSET.
					//
					HeapFree(GetProcessHeap(), 0, pWSAQuerySet);
					pWSAQuerySet = (PWSAQUERYSET) HeapAlloc(GetProcessHeap(),
							HEAP_ZERO_MEMORY,
							ulPQSSize);
					if ( NULL == pWSAQuerySet ) {
						printf("!ERROR! | Unable to allocate memory for WSAQERYSET\n");
						bContinueLookup = FALSE;
					}
				} else {
					printf("=CRITICAL= | WSALookupServiceNext() failed with error code %d\n", iResult);
					bContinueLookup = FALSE;
				}
			}
		}

		//
		// End the lookup service
		//
		WSALookupServiceEnd(hLookup);
	}

CleanupAndExit:
	if ( NULL != pWSAQuerySet ) {
		HeapFree(GetProcessHeap(), 0, pWSAQuerySet);
		pWSAQuerySet = NULL;
	}

	if ( bRemoteDeviceFound ) {
		return(CXN_SUCCESS);
	} else {
		return(CXN_ERROR);
	}
}

//
// Convert a formatted BTH address string to populate a BTH_ADDR (actually a ULONGLONG)
//
// Note: this is an illustration, prefer to use the Winsock library function
//      WSAStringToAddress
//
ULONG AddrStringToBtAddr(IN const char * pszRemoteAddr, OUT BTH_ADDR * pRemoteBtAddr)
{
	int i;
	ULONG		ulAddrData[6], ulRetCode = CXN_SUCCESS;
	BTH_ADDR	BtAddrTemp = 0;

	if ( ( NULL == pszRemoteAddr ) || ( NULL == pRemoteBtAddr ) ) {
		ulRetCode = CXN_ERROR;
		goto CleanupAndExit;
	}

	*pRemoteBtAddr = 0;

	//
	// Populate a 6 membered array of unsigned long integers
	// by parsing the given address in string format
	//
	if (6 != sscanf_s(pszRemoteAddr,
				"%02x:%02x:%02x:%02x:%02x:%02x",
				&ulAddrData[0],&ulAddrData[1],&ulAddrData[2],&ulAddrData[3],&ulAddrData[4],&ulAddrData[5])) {
		ulRetCode = CXN_ERROR;
		goto CleanupAndExit;
	}

	//
	// Construct a BTH_ADDR from the 6 integers stored in the array
	//
	for (i=0; i<6; i++ ) {
		//
		// Extract data from the first 8 lower bits.
		//
		BtAddrTemp = (BTH_ADDR)( ulAddrData[i] & 0xFF );

		//
		// Push 8 bits to the left
		//
		*pRemoteBtAddr = ( (*pRemoteBtAddr) << 8 ) + BtAddrTemp;
	}

CleanupAndExit:
	return(ulRetCode);
}


static
int sbbtwin_open(void * _data) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
    ULONG 		ulRetCode = 0;
    WSADATA		WSAData = {0};
    ULONGLONG 	ululRemoteBthAddr = 0;
    SOCKADDR_BTH	SockAddrBthServer = {0};


	ulRetCode = WSAStartup(MAKEWORD(2, 2), &WSAData);
	if ( 0 != ulRetCode ) { // "zero" per SDK
		if (that->outputErrors) {
			printf("-FATAL- | Unable to initialize Winsock version 2.2\n");
		}
		return -1;
	}

	ulRetCode = AddrStringToBtAddr(that->host, (BTH_ADDR *) &ululRemoteBthAddr);
	if ( CXN_SUCCESS != ulRetCode ) {
		// the address is not a direct MAC address, let's try to use it as a
		// name
		ulRetCode = NameToBthAddr(that->host, (BTH_ADDR *) &ululRemoteBthAddr);
		if ( CXN_SUCCESS != ulRetCode ) {
			if (that->outputErrors) {
				printf("-FATAL- | Unable to find device address for %s\n",that->host);
			}
			return -1;
		}
	}

    //
    // Setting address family to AF_BTH indicates winsock2 to use Bluetooth sockets
    // Port should be set to 0 if ServiceClassId is spesified.
    //
    SockAddrBthServer.addressFamily = AF_BTH;
    SockAddrBthServer.btAddr = (BTH_ADDR) ululRemoteBthAddr;
    SockAddrBthServer.serviceClassId = g_guidServiceClass;
    SockAddrBthServer.port = 0;


	// Create
	that->sock = socket(AF_BTH, SOCK_STREAM, BTHPROTO_RFCOMM);
	if (INVALID_SOCKET == that->sock) {
		if (that->outputErrors) {
            printf("=CRITICAL= | socket() call failed. WSAGetLastError = [%d]\n", WSAGetLastError());
		}
		return -1;
	}

	if ( SOCKET_ERROR == connect(that->sock,
				(struct sockaddr *) &SockAddrBthServer,
				sizeof(SOCKADDR_BTH)) ) {
		if (that->outputErrors) {
			printf("=CRITICAL= | connect() call failed. WSAGetLastError=[%d]\n", WSAGetLastError());
		}
		ulRetCode = -1;
		goto CleanupAndExit;
	}

	{
		// Set non blocking
		u_long nNoBlock = 1;
		ioctlsocket(that->sock, FIONBIO, &nNoBlock);
	}

CleanupAndExit:
	if ( INVALID_SOCKET != that->sock ) {
		closesocket(that->sock);
		that->sock = INVALID_SOCKET;
	}

	return 0;
}

static
int sbbtwin_close(void * _data) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;

	if ( INVALID_SOCKET != that->sock ) {
		closesocket(that->sock);
		that->sock = INVALID_SOCKET;
	}
	return 0;
}

static
int sbbtwin_destroy(void * _data) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
	if (that) {
		free(that->host);
		if (that->sock != INVALID_SOCKET) {
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
	int status = send ( that->sock, msg, size, 0 );
	if ((status == SOCKET_ERROR) && that->outputErrors) {
		printf("send() call failed w/socket = [0x%X], szData = [%p], dataLen = [%d]. WSAGetLastError=[%d]\n", 
				that->sock, msg, size, WSAGetLastError());
	}
	return status;
}

static
int sbbtwin_receive(void * _data, unsigned char *msg, size_t size) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
	int status = recv ( that->sock, msg, size , 0 );
	if ((status == SOCKET_ERROR) && that->outputErrors) {
		printf("recv() call failed. WSAGetLastError=[%d]\n", WSAGetLastError());
	}
	if ((status == 0) && that->outputErrors) {
		printf("recv() call failed. The socket has been closed\n");
	}
	return status;
}

#ifndef __GNUC__
#define EPOCHFILETIME (116444736000000000i64)
#else
#define EPOCHFILETIME (116444736000000000LL)
#endif

static double now() {
    FILETIME        ft;
    LARGE_INTEGER   li;
	__int64         t;

	GetSystemTimeAsFileTime(&ft);
	li.LowPart  = ft.dwLowDateTime;
	li.HighPart = ft.dwHighDateTime;
	t  = li.QuadPart;       /* In 100-nanosecond intervals */
	t -= EPOCHFILETIME;     /* Offset to the Epoch time */
	return t * 1e-7;
}


static
int sbbtwin_sendall(void * _data, const unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	unsigned int sentbytes;
	double t1ms,t2ms,dtms,maxms;
	maxms = timeout_ms * 1e-3;
	t1ms = now();
	sentbytes = 0;
	while (sentbytes < size)
	{
		int r = sbbtwin_send (_data, msg+sentbytes, size-sentbytes);
		if (r < 0) {return r;}
		if (r > 0) sentbytes += r;
		//printf("Sent %d ",sentbytes);
		t2ms = now();
		dtms = t2ms - t1ms;
		//printf("%f %f dtms : %f/%f\n",t1ms,t2ms,dtms,maxms);
		
		if ((timeout_ms>0)&&(dtms >= maxms)) break;
	}
	//printf(" Finally sent %d bytes\n",sentbytes);
	return (sentbytes == size)?0:-1;
}

static
int sbbtwin_waitbuffer(void * _data,  unsigned char *msg, size_t size, unsigned int timeout_ms) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
	fd_set rfs;
	unsigned int readbytes;
	double t1ms,t2ms,dtms,maxms;
	struct timeval to = {0,0};
	maxms = timeout_ms*1e-3;
	to.tv_sec = (timeout_ms/1000);
	to.tv_usec = (timeout_ms%1000) * 1000;
	readbytes = 0;
	t1ms = now();
	while (readbytes < size)
	{
		int r;
		to.tv_sec = (timeout_ms/1000);
		to.tv_usec = (timeout_ms%1000) * 1000;
		FD_ZERO(&rfs);FD_SET(that->sock,&rfs);
		r = select(that->sock+1,&rfs,NULL,NULL,&to);
		if (r < 1) return -1;
		r = sbbtwin_receive (_data, msg+readbytes, size-readbytes );
		if (r>0) readbytes += r;
		if (r < 1) return -1;
		// printf("Rec %d\n",readbytes);

		t2ms = now();
		dtms = t2ms - t1ms;
		if ((timeout_ms>0) && (dtms >= maxms)) break;
		dtms = maxms - dtms;
		to.tv_sec = (unsigned int)(dtms);
		to.tv_usec = (unsigned int)((dtms - to.tv_sec)*1e6);
	}
	// printf("Received %d bytes out %d\n",readbytes,size);
	return (readbytes == size)?0:-1;
}

static
int sbbtwin_waitdata(void * _data, unsigned int timeout_ms) 
{
	struct SBBTWinChannelData *that = (struct SBBTWinChannelData*)_data;
	fd_set rfs;
	struct timeval to = {0,0};
	int r;
	to.tv_sec = (timeout_ms/1000);
	to.tv_usec = (timeout_ms%1000) * 1000;
	FD_ZERO(&rfs);FD_SET(that->sock,&rfs);
	r = select(that->sock+1,&rfs,NULL,NULL,(timeout_ms>0)?(&to):NULL);
	return (r >= 1)?0:-1;
}

static
int sbbtwin_flush(void * _data) 
{
	unsigned char trash[32];
	while (sbbtwin_receive(_data,trash,32)>0);
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

