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

#ifndef HEX_FILE_H
#define HEX_FILE_H

#include <stdlib.h>
#include <assert.h>

#include <string>
#include <map>

#include "PacketStore.h"

class MemoryChunk {
	public:
		static const unsigned int ChunkSize = 0x20000;
	protected:
		unsigned char *data;
		unsigned int address;
		unsigned int len;
	public:
		MemoryChunk(unsigned int addr = 0) {
			data = (unsigned char*)malloc(ChunkSize);
			assert(data);
			memset(data,0xFF,ChunkSize);
			address = addr;
			len = 0;
		}

		MemoryChunk(const MemoryChunk & m) {
			data = (unsigned char*)malloc(ChunkSize);
			assert(data);
			memcpy(data,m.data,ChunkSize);
			address = m.address;
			len = m.len;
		}

		~MemoryChunk() {
			free(data);
			data = NULL;
			len = 0;
		}

		unsigned int getAddress() const {
			return address;
		}

		unsigned int length() const {
			return len;
		}

		unsigned char & operator[](unsigned int i) {
			assert(i<ChunkSize);
			len = std::max(len,i+1);
			return data[i];
		}

		const unsigned char & operator[](unsigned int i) const {
			assert(i<len);
			return data[i];
		}

		void writeToFile(FILE * fp) {
			fprintf(fp,"Base address: %d (length: %d bytes) %p",address,len,this);
			for(unsigned int j=0;j<len;j++)
			{
				if (j % 16==0)
					fprintf(fp,"\n");
				fprintf(fp,"%02X ",data[j]);
			}
			fprintf(fp,"\n");
		}
};

class HexFile : public std::map<unsigned int,MemoryChunk*>
{
	protected:
		MemoryChunk* getChunk(unsigned int address);
		const MemoryChunk* getChunk(unsigned int address) const;
	public:
		HexFile() {}
		~HexFile();

		int loadFile(const std::string & fname);

		int writeFile(const std::string & fname);

		int setBootLoaderAddress(unsigned int address);

		unsigned int getProgramAddress() const;

#if 0
		BTPacket createProgramAddressPacket(unsigned int lastPageAddress, 
				unsigned int startAddress,
				unsigned int robotid);
#endif

		int createErasePackets(PacketStore & store);
		size_t createPackets(PacketStore & store,
				unsigned int bootloaderStart, unsigned int bootloaderEnd);
};



#endif // HEX_FILE_H
