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

#ifndef PACKET_STORE_H
#define PACKET_STORE_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <list>
#define WORD_PER_WRITE 64
#define WORD_SIZE 3
#define HEADER_LEN 4
#define CRC_LEN 1
#define PACKET_LEN (HEADER_LEN+WORD_PER_WRITE*WORD_SIZE+CRC_LEN)


struct BTPacket {
	unsigned char data[PACKET_LEN]; 
	BTPacket() {
		memset(data,0xFF,PACKET_LEN);
	}
	BTPacket(unsigned int address, bool erase=false) {
		memset(data,0xFF,PACKET_LEN);
		data[0] = address; address = address >> 8;
		data[1] = address; address = address >> 8;
		data[2] = address; 
		data[3] = erase?0xFE:0x60;
	}

	void setCheckSum() {
		unsigned int i,sum=0;
		for (i=0;i<PACKET_LEN-1;i++) {
			sum += data[i];
		}
		sum &= 0xFF;
		data[i] = (sum==0)?(0):(0x100-sum);
	}

	~BTPacket() {}

	unsigned char * operator[](unsigned int i) {
		return data+(HEADER_LEN+i*WORD_SIZE);
	}

	int writeToFile(FILE * fp) const;

	unsigned long getAddress() const {
		unsigned long a = data[2];
		a = a << 8; a |= data[1];
		a = a << 8; a |= data[0];
		return a;
	}

	bool isErase() const {
		return data[3] == 0xFE;
	}
};

class PacketStore : public std::list<BTPacket> {
	protected:

	public:
		PacketStore() {}
		~PacketStore() {}

		int writeToFile(const std::string & fname) const;
};



#endif // PACKET_STORE_H
