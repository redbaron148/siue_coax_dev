/*
	coaxuploadbt.cpp
	
	libbluetooth based uploader for coax
	
	Based on:
	-	epuckupload.c
		Created by Olivier Michel on 12/15/06.
		(inpired from epuckupload perl script by Xavier Raemy and Thomas Lochmatter)
		Copyright 2006 Cyberbotics Ltd. All rights reserved.
		
	-	libbluetooth samples from Internet

	- epuckloadbt.cpp
		(c) 2007 Stephane Magnenat (stephane at magnenat dot net) - Mobots group - LSRO1 - EPFL

	- coaxuploadbt.cpp
		(c) 2009 Cedric Pradalier - ETH Zurich
*/

#include <cstdlib>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sbcom/sbchannel.h>

#include "PacketStore.h"
#include "HexFile.h"

const unsigned programVersion = 2;
const char *programName = "epuck bluetooth uploader (using libbluetooth)";
const char *copyrightInfo = "(c) 2006 - 2009 Olivier Michel, Stephane Magnenat, Cedric Pradalier";

/* Send packets to file descriptor fd */
int sendPackets(SBChannel *channel, const PacketStore & store)
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
	sbChannelFlush(channel);
	// wait user input
	printf("Press enter and then reset the robot\n");
	getchar();
	
	// wait for 50 times 100 ms
	bool found = false;
	buffer[0]=0xC1;
	for (unsigned i = 0; i < 100; i++)
	{
		sbChannelSend(channel,buffer,1);
		if (retval = sbChannelWaitData(channel,100)) {
			found = true;
		}
#ifdef DEBUG
		printf("%02d - Select: %d\n",i,retval);
#else
		printf(".");fflush(stdout);
#endif
		
		if (found) break;
	}
	printf("\n");
	
	// if no answer, quit
	if (!found)
	{
		fprintf(stderr,"\n\rError, bootloader did not answered\n");
		return false;
	}
	
	// checking ack
	int ok = -1;
	int n = 0;
	if (sbChannelWaitBuffer(channel,buffer,2,100)) {
		fprintf(stderr,"\n\rError, bootloader did not answered 2 bytes\n");
		return false;
	}
#ifdef DEBUG
	printf("Ack %d bytes: ",n);
#endif
	for (int i=0;i<n;i++) {
		if (buffer[i]=='q' && buffer[i+1]=='K') {
			ok = 0;
		}
#ifdef DEBUG
		printf("%02X/%c %d\n",buffer[i],buffer[i],ok);
#endif
	}
#ifdef DEBUG
	printf("\n");
#endif

	if (ok!=0)
	{
		fprintf(stderr,"\n\rError, no ack received from bootloader\n");
		return false; // fail, we didn't receive the ack from bootloader
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
			if (sbChannelSendAll(channel,it->data+n,std::min(PACKET_LEN-n,8),500)) {
				fprintf(stderr,"\n\rError, while sending packet\n");
				return false; 
			}
			// printf("%d ",n);fflush(stdout);
			Sleep(1);
		}
#endif
#ifdef DEBUG
		printf("<");fflush(stdout);
#endif
		while (1) {
			bool end = false;
			buffer[0] = 0;
			if (sbChannelWaitBuffer(channel,buffer,1,100)) {
				fprintf(stderr,"\n\rError, while receiving packet ack\n");
				return false; 
			}
			switch(buffer[0])
			{
				case 'K': 
					printf("#"); 
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
	if (sbChannelSendAll(channel,buffer,4,100)) {
		fprintf(stderr,"\n\rError, while sending last packet\n");
		return false; 
	}
	return ok;
}

#define BOOTLOADER_START 0x00400
#define PROGRAM_START 0x02000

int main(int argc,char *argv[])
{
	int robot_id=0;
	HexFile hf;
	PacketStore store;
	
	printf("%s version %d\n%s\n",programName,programVersion,copyrightInfo);
	if (argc!=3)
	{
		fprintf(stderr,"Error, wrong number of arguments, usage:\n\t%s FILE COMPORT\n",argv[0]);
		return 1;
	}
	
	if (hf.loadFile(argv[1])) {
		fprintf(stderr,"Error, cannot open file: %s \n",argv[1]);
		return 2;
	}

	unsigned int programAddress = hf.getProgramAddress();
	printf("Program located at %06X\n", programAddress);
	hf.setBootLoaderAddress(BOOTLOADER_START);
	store.clear();
	// Create all the packets to be written
	hf.createPackets(store,BOOTLOADER_START,PROGRAM_START-1);
	// From the list of packets to be written, extract the list of pages
	// to erase
	hf.createErasePackets(store);
	hf.writeFile("coaxmemory.txt");
	store.writeToFile("coaxpackets.txt");

	SBChannel channel;
	if (sbChannelCreateBTWin(&channel,argv[2],115200)) {
		fprintf(stderr, "Failed to open bluetooth port\n");
		return -1;
	}
#if 1
	if (!sendPackets(&channel, &store)) {
		fprintf(stderr, "Error while sending packets\n");
		return -1;
	}
#endif
	sbChannelDestroy(&channel);
	return 0;
}
