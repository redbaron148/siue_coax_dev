/*
	coaxuploadbt.cpp
	
	libbluetooth based uploader for coax
	
	Based on:
	-	Coaxupload.c
		Created by Olivier Michel on 12/15/06.
		(inpired from Coaxupload perl script by Xavier Raemy and Thomas Lochmatter)
		Copyright 2006 Cyberbotics Ltd. All rights reserved.
		
	-	libbluetooth samples from Internet

	- Coaxloadbt.cpp
		(c) 2007 Stephane Magnenat (stephane at magnenat dot net) - Mobots group - LSRO1 - EPFL

	- coaxuploadbt.cpp
		(c) 2009 Cedric Pradalier - ETH Zurich
*/

#include <cstdlib>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>

#include "PacketStore.h"
#include "HexFile.h"
#include "bootloader.h"

const unsigned programVersion = 2;
const char *programName = "coax bluetooth uploader (using libbluetooth)";
const char *copyrightInfo = "(c) 2006 - 2009 Olivier Michel, Stephane Magnenat, Cedric Pradalier";

// #define DEBUG

/* Send packets to file descriptor fd */
int send_packets(int fd, const PacketStore & store)
{
	// send reset signal: R, ignore answered chars
	// send bootloader start char: 0xC1
	// receive 'q' and 'K' answer
	// send packet
	// receive 'K' answer
	// ...
	// send packet
	// receive 'K' answer
	// send termination: 0x95, 0x00, 0x00, 0xFF
	unsigned char buffer[2048];
	
	// eat input
	int retval;
	
	// wait user input
	printf("Press enter and then reset on the robot\n");
	getchar();
	
	// wait for 50 times 100 ms
	int ok = -1;
	int n = 0;
	bool found = false;
	for (unsigned i = 0; i < 100; i++)
	{
		struct timeval timeout;
		buffer[0]=0xC1;
		timeout.tv_sec=0;  // 100 ms timeout
		timeout.tv_usec=200000; // (to leave time to establish the connection)

		fd_set readfds;
		FD_ZERO(&readfds);
		FD_SET(fd,&readfds);

		// First, absorb anything in the buffer.
		while (1)
		{
			struct timeval timeout;
			timeout.tv_sec=0;
			timeout.tv_usec=0;
			fd_set readfds;
			FD_ZERO(&readfds);
			FD_SET(fd,&readfds);
			retval = select(fd+1,&readfds,NULL,NULL,&timeout);
			if (retval != 0) {
				int n = 0;
				n = read(fd,buffer,64);
#ifdef DEBUG
				printf("flushing %d bytes: ",n);
				for (int i=0;i<n;i++) {
					printf("%02X ",buffer[i]);
				}
				printf("\n");
#endif
			} else {
				break;
			}
		}
		usleep(200000);

		// Then send the trigger byte
		write(fd,buffer,1);
		retval = select(fd+1,&readfds,NULL,NULL,&timeout);
#ifdef DEBUG
		printf("%02d - Select: %d\n",i,retval);
#else
		printf(".");fflush(stdout);
#endif
		if (retval == 0) {
			// timeout
			continue;
		}

		found = false;
		n = 0;
		// checking ack
		while (n<2) {
			n += read(fd,buffer+n,64);
		}
#ifdef DEBUG
		printf("Ack %d bytes: ",n);
#endif
		for (int i=0;i<n-1;i++) {
			if (buffer[i]=='q' && buffer[i+1]=='K') {
				found = true;
			}
#ifdef DEBUG
			printf("%02X/%c %d\n",buffer[i],buffer[i],ok);
#endif
		}
#ifdef DEBUG
		printf("\n");
#endif

		if (found) {
			break;
		}
	}
	printf("\n");
	
	// if no answer, quit
	if (!found)
	{
		fprintf(stderr,"\n\rError, bootloader did not answered\n");
		return false;
	}
	
	
	// uploading code
	ok = 0;
	printf("\rUploading: \n");
	PacketStore::const_iterator it;
	for(it = store.begin(); it != store.end(); it ++) {
#ifdef DEBUG
		it->writeToFile(stdout);
		printf(">");fflush(stdout);
#endif
#if 0
		n = write(fd,it->data,PACKET_LEN);
		printf("%d",n);
#else
		n = 0;
		while (n < PACKET_LEN) {
			n += write(fd,it->data+n,std::min(PACKET_LEN-n,8));
			// printf("%d ",n);fflush(stdout);
			usleep(500);
		}
#endif
#ifdef DEBUG
		printf("<");fflush(stdout);
#endif
		while (1) {
			bool end = false;
			buffer[0] = 0;
			n=read(fd,buffer,64);
			for (int i=0;i<n;i++) {
				switch(buffer[i])
				{
					case 'K': 
						printf(it->isErase()?"X":"#"); 
						end=true;
						break; // all right
					case 'N': 
						printf("N"); 
						ok = -1;
						end=true;
						break; // checksum error
					case '!': 
						printf("!"); 
						break;
					case '+': 
						printf("+"); 
						break;
					default:  
						printf("?%02X",buffer[0]); 
						break; // unknown error
				}
			}
			if (end) break;
		}
#ifdef DEBUG
		printf("\n");
#endif
		fflush(stdout);
	}
	buffer[0]=0x95;
	buffer[1]=0x00;
	buffer[2]=0x00;
	buffer[3]=0xff;
	write(fd,buffer,4);
	return ok;
}

bool connectToCoax(const bdaddr_t *ba, const PacketStore & store)
{
	// set the connection parameters (who to connect to)
	struct sockaddr_rc addr;
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	addr.rc_bdaddr = *ba;
	//memcpy(addr.rc_bdaddr, ba, sizeof(bdaddr_t));
	
	// allocate a socket
	int rfcommSock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	
	// connect to server
	int status = ::connect(rfcommSock, (struct sockaddr *)&addr, sizeof(addr));
	
	if (status == 0)
	{
		unsigned trial = 0;
		while (true)
		{
			if (send_packets(rfcommSock,store))
				std::cerr << "Upload failed (" << trial << ") . Trying again. Press CTRL-C to quit" << std::endl;
			else
				break;
			trial++;
		}
	}
	else
	{
		std::cerr << "Error, can't connect to rfcomm socket" << std::endl;
	}
	
	close(rfcommSock);
	
	return status == 0;
}

bool scanBluetoothAndUpload(int robotId, const PacketStore & store)
{
	// open device
	int devId = hci_get_route(NULL);
	if (devId < 0)
	{
		std::cerr << "Error, can't get bluetooth adapter ID" << std::endl;
		return false;
	}
	
	// open socket
	int sock = hci_open_dev(devId);
	if (sock < 0)
	{
		std::cerr << "Error,can't open bluetooth adapter" << std::endl;
		return false;
	}

	// query
	std::cout << "Scanning bluetooth:" << std::endl;
	//int length  = 8; /* ~10 seconds */
	int length  = 4; /* ~5 seconds */
	inquiry_info *info = NULL;
	// device id, query length (last 1.28 * length seconds), max devices, lap ??, returned array, flag
	int devicesCount = hci_inquiry(devId, length, 255, NULL, &info, 0);
	if (devicesCount < 0)
	{
		std::cerr << "Error, can't query bluetooth" << std::endl;
		close(sock);
		return false;
	}
	
	// print devices
	bool found = false;
	for (int i = 0; i < devicesCount; i++)
	{
		char addrString[19];
		char addrFriendlyName[256];
		ba2str(&(info+i)->bdaddr, addrString);
		if (hci_read_remote_name(sock, &(info+i)->bdaddr, 256, addrFriendlyName, 0) < 0)
			strcpy(addrFriendlyName, "[unknown]");
		printf("\t%s %s\n", addrString, addrFriendlyName);
		if (strncmp("CoaX_", addrFriendlyName, 5) == 0)
		{
			int id,r;
			r = sscanf(addrFriendlyName, "CoaX_%d", &id);
			if ((r==1) && (id == robotId))
			{
				printf("Contacting Coax %d\n",id);
				found = connectToCoax(&(info+i)->bdaddr,store);
				break;
			}
		}
	}
	
	if (!found)
		std::cerr << "Error: Coax " << robotId << " not found" << std::endl;
	
	free(info);
	close(sock);
	
	return found;
}


int main(int argc,char *argv[])
{
	int robot_id=0;
	HexFile hf;
	PacketStore store;
	
	printf("%s version %d\n%s\n",programName,programVersion,copyrightInfo);
	if (argc!=3)
	{
		fprintf(stderr,"Error, wrong number of arguments, usage:\n\t%s FILE ROBOT_ID\n",argv[0]);
		return 1;
	}
	
	if (hf.loadFile(argv[1])) {
		fprintf(stderr,"Error, cannot open file: %s \n",argv[1]);
		return 2;
	}
	sscanf(argv[2],"%d",&robot_id);

	unsigned int programAddress = hf.getProgramAddress();
	printf("Program located at %06X\n", programAddress);
	hf.setBootLoaderAddress(BOOTLOADER_START);
	store.clear();
	// Create all the packets to be written
	size_t uploadSize = hf.createPackets(store,BOOTLOADER_START,FLASH_END-1);
	size_t npackets = store.size();
	printf("Upload size: %d packets %'d words (%'d bytes)\n",npackets, uploadSize,uploadSize*WORD_SIZE);
	// From the list of packets to be written, extract the list of pages
	// to erase
	hf.createErasePackets(store);
	printf("Added %d erase packets\n",store.size()-npackets);
	hf.writeFile("coaxmemory.txt");
	store.writeToFile("coaxpackets.txt");
#if 1
	if (scanBluetoothAndUpload(robot_id,store))
	{
		printf("\nUpload success :-)\n");
		return 0;
	}
	else
	{
		printf("\nUpload failure :-(\n");
		return 3;
	}
#endif
}
