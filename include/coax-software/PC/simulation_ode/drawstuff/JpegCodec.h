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

#ifndef JPEG_CODEC_H
#define JPEG_CODEC_H

extern "C" {
#include <jpeglib.h>
}

    
class JpegCodec
{
	public:
		typedef enum {cmAuto, cmGray, cmRGB, cmYUV} ColorSpace;
	protected :
		unsigned int size;
		unsigned int width,height;
		ColorSpace inputColorSpace, outputColorSpace;

		unsigned char * buffer;

		int quality; // between 0 and 100
	public :
		JpegCodec(int qual=75);
		void setQuality(int qual);
		int getQuality() const {return quality;}

		virtual ~JpegCodec();

		bool encode(unsigned char * src, 
				unsigned int width,
				unsigned int height);

		bool encode(unsigned char * src, 
				unsigned int width,
				unsigned int height, 
				const char * filename);

		void setInputColorSpace(ColorSpace cspace);
		void setOutputColorSpace(ColorSpace cspace);

		unsigned int bsize() {return size;}
		bool resizebuffer(unsigned int size);
		const unsigned char * getBuffer() const {return buffer;}
		unsigned char * getBuffer() {return buffer;}
		unsigned int getWidth() {return width;}
		unsigned int getHeight() {return height;}
};




#endif // JPEG_CODEC_H
