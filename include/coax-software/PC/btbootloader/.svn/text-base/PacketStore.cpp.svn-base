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

#include <string>

#include "PacketStore.h"

int BTPacket::writeToFile(FILE * fp) const
{
	unsigned int i;
	fprintf(fp,"%02X%02X%02X  %02X  ",
			data[2],data[1],data[0], data[3]);
	for (i=HEADER_LEN;i<PACKET_LEN-CRC_LEN;i++) {
		fprintf(fp,"%02X ",data[i]);
	}
	fprintf(fp,"| %02X\n",data[i]);
	return 0;		
}


int PacketStore::writeToFile(const std::string & fname) const
{
	const_iterator it;
	FILE * fp = fopen(fname.c_str(),"w");
	if (!fp) {
		perror("PacketStore::writeToFile");
		return -1;
	}
	for (it=begin();it!=end();it++) {
		it->writeToFile(fp);
	}
	fclose(fp);
	return 0;
}
