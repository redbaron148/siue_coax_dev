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

#include <set>

#include "HexFile.h"

HexFile::~HexFile()
{
	iterator it;
	for (it=begin();it!=end();it++) {
		// delete it->second;
	}
	clear();
}

int HexFile::loadFile(const std::string & fname)
{
	char line[1024];
	char hex[5];
	int i,j,k,value,line_counter;
	unsigned char len;
	unsigned short int address;
	unsigned char type;
	unsigned char checksum;
	MemoryChunk *m=NULL;

	FILE *fd=fopen(fname.c_str(),"r");
	if (fd==NULL)
		return -1;
	line_counter=0;
	while (fscanf(fd,"%64s",line)==1)
	{
		line_counter++;
		i=1;
		hex[0]=line[i++];
		hex[1]=line[i++];
		hex[2]='\0';
		sscanf(hex,"%x",&value);
		len=(unsigned char)value;
		hex[0]=line[i++];
		hex[1]=line[i++];
		hex[2]=line[i++];
		hex[3]=line[i++];
		hex[4]='\0';
		sscanf(hex,"%x",&value);
		address=(unsigned short int)value;
		hex[0]=line[i++];
		hex[1]=line[i++];
		hex[2]='\0';
		sscanf(hex,"%x",&value);
		type=(unsigned char)value;
		j=i; /* start of data chunk */
		k=strlen(&line[j])/2-1;
		if (k!=len)
		{
			fprintf(stderr,"wrong data length on line %d, ignored",line_counter);
			continue;
		}
		i+=2*len;
		hex[0]=line[i++];
		hex[1]=line[i--]; // return to previous position with i
		// 0 already set for hex[2]
		sscanf(hex,"%x",&value);
		checksum=(unsigned char)value;
		// printf("%s (len=%d, address=%x, type=%d checksum=%x)\n",line,len,address,type,checksum);
		if (type==0 && m!=NULL)
		{
			for(k=0;k<len;k++)
			{
				hex[0]=line[j++];
				hex[1]=line[j++];
				// 0 already set for hex[2]
				sscanf(hex,"%x",&value);
				(*m)[address+k]=(unsigned char)value;
			}
#if 0
			printf("New Data at %04X in %p / New len %d: ",address,m,m->length());
			for(k=0;k<len;k++) {
				printf("%02X ",(*m)[address+k]);
			}
			printf("\n");
#endif

		}
		else if (type==4)
		{
			hex[0]=line[j++];
			hex[1]=line[j++];
			hex[2]=line[j++];
			hex[3]=line[j++];
			hex[4]='\0';
			sscanf(hex,"%x",&value);
			if (value<4)
			{
				value*=0x10000;
				m = getChunk(value);
			}
			else
				m=NULL;
		}
	}
	fclose(fd);
	return 0;
}

int HexFile::writeFile(const std::string & fname)
{
	FILE *fd=fopen(fname.c_str(),"w");
	iterator it;
	if (fd==NULL) return -1;
	
	for(it=begin();it!=end();it++) {
		it->second->writeToFile(fd);
	}
	fclose(fd);
	return 0;
}


const MemoryChunk* HexFile::getChunk(unsigned int address) const
{
	const_iterator it = find(address);
	if (it == end()) {
		return NULL;
	} else {
		return it->second;
	}
}


MemoryChunk* HexFile::getChunk(unsigned int address)
{
	iterator it = find(address);
	if (it == end()) {
		MemoryChunk *m = new MemoryChunk(address);
		insert(std::pair<unsigned int,MemoryChunk*>(address,m));
		return m;
	} else {
		return it->second;
	}
}

int HexFile::setBootLoaderAddress(unsigned int address)
{
	// Encoding of goto address/24 : 
	// 0x04<addr:0-7><addr:8-15>
	// 0x0000<addr:16:23>
	MemoryChunk *m = getChunk(0);
	(*m)[0] = address; address = address >> 8;
	(*m)[1] = address; address = address >> 8;
	(*m)[2] = 0x04;
	(*m)[3] = 0x00; // ignored
	(*m)[4] = address;
	(*m)[5] = 0x00;
	(*m)[6] = 0x00;
	(*m)[7] = 0x00; // ignored
	return 0;
}


#if 0
BTPacket HexFile::createProgramAddressPacket(
		unsigned int lastPageAddress, 
		unsigned int startAddress,
		unsigned int robotid)
{
	// This packet will go at the beginning of the bootloader.
	// It has to be 32 words, we place it so that the last 4 words overlap with
	// the bootloader.
	BTPacket P(lastPageAddress);
	// CLR.W W0
	P[0][0] = 0x00;
	P[0][1] = 0x00;
	P[0][2] = 0xEB;
	
	// MOV.W #robotid,W0
	P[1][0] = (robotid << 4) & 0xF0 ; robotid = robotid >> 4;
	P[1][1] = robotid;                robotid = robotid >> 8;
	P[1][2] = (robotid & 0x0F) | 0x20;

	// Goto address
	// Encoding of goto address/24 : 
	// 0x04<addr:0-7><addr:8-15>
	// 0x0000<addr:16:23>
	P[2][0] = startAddress; startAddress = startAddress >> 8;
	P[2][1] = startAddress; startAddress = startAddress >> 8;
	P[2][2] = 0x04;

	P[3][0] = startAddress;
	P[3][1] = 0x00;
	P[3][2] = 0x00;

	P.setCheckSum();

	return P;
}
#endif


size_t HexFile::createPackets(PacketStore & store, unsigned int bootloaderStart, unsigned int bootloaderEnd)
{
	size_t uploadSize = 0;
	unsigned int k,addr;
	const_iterator it;

	//printf("Creating packet except for addresses between %06X and %06X\n",
	// 	   bootloaderStart,bootloaderEnd);	
	for (it=begin();it!=end();it++) {
		addr=0;
		while(addr<it->second->length())
		{
			unsigned int address = (it->second->getAddress()+addr)/2;
			BTPacket P(address,false);
			for(k=0;k<WORD_PER_WRITE;k++)
			{
				unsigned char *b = P[k]; 
				if (addr < it->second->length())
				{
					b[0] = (*it->second)[addr++];
					b[1] = (*it->second)[addr++];
					b[2] = (*it->second)[addr++];
				} else {
					break;
				}
				addr++;
			}
			P.setCheckSum();
			// HACK to avoid destroying the bootloader table
			if ((address<bootloaderStart) || (address>bootloaderEnd)) {
				store.push_front(P);
				uploadSize += WORD_PER_WRITE;
			} else {
				// printf("Rejecting %06X\n",address);
			}
		}
	}
	return uploadSize;
}

int HexFile::createErasePackets(PacketStore & store)
{
	PacketStore::const_iterator it;
	std::set<unsigned long,std::less<unsigned long> > erasepage;
	std::set<unsigned long,std::less<unsigned long> >::const_iterator epit;

	for (it=store.begin();it!=store.end();it++) {
		erasepage.insert(it->getAddress() & 0xFFFC00);
	}

	for (epit=erasepage.begin();epit!=erasepage.end();epit++) {
		BTPacket P(*epit,true);
		P.setCheckSum();
		store.push_front(P);
	}
	return 0;
}


unsigned int HexFile::getProgramAddress() const
{
	const MemoryChunk *m = getChunk(0);
	unsigned char opcode = (*m)[2];
	if (opcode != 0x04) {
		// 0x04 is GOTO
		return 0x000000;
	}
	unsigned int address = (*m)[4];
	address = (address << 8) | (*m)[1];
	address = (address << 8) | (*m)[0];
	return address;
}



