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

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

extern "C" {
#include <jpeglib.h>
}
#include <string.h>
#include <setjmp.h>

#include "JpegCodec.h"

JpegCodec::JpegCodec(int qual)
{
	width = height = 0;
	outputColorSpace = cmAuto;
	inputColorSpace = cmAuto;
	size = 0;
	buffer = NULL;
	setQuality(qual);
}

JpegCodec::~JpegCodec()
{
	//printf("~JpegCodec\n");
	free(buffer);
	buffer = NULL;
	
}

void JpegCodec::setQuality(int qual)
{
	if (qual <= 0) qual = 75;
	if (qual > 100) qual = 100;
	quality = qual;
}


struct my_error_mgr {
	struct jpeg_error_mgr pub;	/* "public" fields */

	jmp_buf setjmp_buffer;	/* for return to caller */
};

typedef struct my_error_mgr * my_error_ptr;

/*
 * Here's the routine that will replace the standard error_exit method:
 */

void my_error_exit (j_common_ptr cinfo)
{
	/* cinfo->err really points to a my_error_mgr struct, so coerce pointer */
	my_error_ptr myerr = (my_error_ptr) cinfo->err;

	/* Always display the message. */
	/* We could postpone this until after returning, if we chose. */
	(*cinfo->err->output_message) (cinfo);

	/* Return control to the setjmp point */
	longjmp(myerr->setjmp_buffer, 1);
}


/* Expanded data destination object for mem output */

#define OUTPUT_BUF_SIZE  4096	/* choose an efficiently fwrite'able size */
typedef struct {
  struct jpeg_destination_mgr pub; /* public fields */

  size_t buffersize, bufferpos;
  JpegCodec *codec;		/* start of buffer */
  JOCTET * wbuffer;

} my_destination_mgr;

typedef my_destination_mgr * my_dest_ptr;

/*
 * Initialize destination --- called by jpeg_start_compress
 * before any data is actually written.
 */

METHODDEF(void)
init_destination (j_compress_ptr cinfo)
{
  my_dest_ptr dest = (my_dest_ptr) cinfo->dest;

  /* Allocate the output buffer --- it will be released when done with image */
  dest->wbuffer = (JOCTET *)
      (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_IMAGE,
				  OUTPUT_BUF_SIZE * sizeof(JOCTET));

  dest->pub.next_output_byte = dest->wbuffer;
  dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;
  dest->bufferpos = 0;
}


/*
 * Empty the output buffer --- called whenever buffer fills up.
 *
 * In typical applications, this should write the entire output buffer
 * (ignoring the current state of next_output_byte & free_in_buffer),
 * reset the pointer & count to the start of the buffer, and return TRUE
 * indicating that the buffer has been dumped.
 *
 * In applications that need to be able to suspend compression due to output
 * overrun, a FALSE return indicates that the buffer cannot be emptied now.
 * In this situation, the compressor will return to its caller (possibly with
 * an indication that it has not accepted all the supplied scanlines).  The
 * application should resume compression after it has made more room in the
 * output buffer.  Note that there are substantial restrictions on the use of
 * suspension --- see the documentation.
 *
 * When suspending, the compressor will back up to a convenient restart point
 * (typically the start of the current MCU). next_output_byte & free_in_buffer
 * indicate where the restart point will be if the current call returns FALSE.
 * Data beyond this point will be regenerated after resumption, so do not
 * write it out when emptying the buffer externally.
 */

#if 0
static void printhex(unsigned char * b, unsigned int len) 
{
	unsigned int i;
	for (i=0;i<len;i++) {
		printf("%02X ",(unsigned int)b[i]);
		if ((i%16)==0) printf("\n");
	}
	printf("\n");
}
#endif
	

METHODDEF(boolean)
empty_output_buffer (j_compress_ptr cinfo)
{
  my_dest_ptr dest = (my_dest_ptr) cinfo->dest;
  dest->codec->resizebuffer(dest->codec->bsize() + 10*OUTPUT_BUF_SIZE);

  // printhex(dest->wbuffer,OUTPUT_BUF_SIZE);

  memcpy(dest->codec->getBuffer() + dest->bufferpos, 
		  dest->wbuffer,OUTPUT_BUF_SIZE);

  dest->bufferpos += OUTPUT_BUF_SIZE;
  dest->pub.next_output_byte = dest->wbuffer;
  dest->pub.free_in_buffer = OUTPUT_BUF_SIZE;

  return TRUE;
}


/*
 * Terminate destination --- called by jpeg_finish_compress
 * after all data has been written.  Usually needs to flush buffer.
 *
 * NB: *not* called by jpeg_abort or jpeg_destroy; surrounding
 * application must deal with any cleanup that should happen even
 * for error exit.
 */

METHODDEF(void)
term_destination (j_compress_ptr cinfo)
{
  my_dest_ptr dest = (my_dest_ptr) cinfo->dest;
  size_t datacount = OUTPUT_BUF_SIZE - dest->pub.free_in_buffer;
  //printhex(dest->wbuffer,datacount);

  /* Write any data remaining in the buffer */
  if (datacount > 0) {
	  dest->codec->resizebuffer(dest->codec->bsize() + datacount);
	  memcpy(dest->codec->getBuffer() + dest->bufferpos, 
			  dest->wbuffer,datacount);

	  dest->bufferpos += datacount;
  }
}


/*
 * Prepare for output to a mem stream.
 * The caller must have already opened the stream, and is responsible
 * for closing it after finishing compression.
 */

GLOBAL(void)
jpeg_mem_dest (j_compress_ptr cinfo, JpegCodec *that)
{
  my_dest_ptr dest;

  /* The destination object is made permanent so that multiple JPEG images
   * can be written to the same file without re-executing jpeg_stdio_dest.
   * This makes it dangerous to use this manager and a different destination
   * manager serially with the same JPEG object, because their private object
   * sizes may be different.  Caveat programmer.
   */
  if (cinfo->dest == NULL) {	/* first time for this JPEG object? */
    cinfo->dest = (struct jpeg_destination_mgr *)
      (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
				  sizeof(my_destination_mgr));
  }

  dest = (my_dest_ptr) cinfo->dest;
  dest->pub.init_destination = init_destination;
  dest->pub.empty_output_buffer = empty_output_buffer;
  dest->pub.term_destination = term_destination;
  dest->wbuffer = NULL;
  dest->codec = that;
  dest->bufferpos = 0;
}

bool JpegCodec::encode(unsigned char * isrc, 
				unsigned int width,
				unsigned int height)
{
	/* This struct contains the JPEG compression parameters and pointers to
	 * working space (which is allocated as needed by the JPEG library).
	 * It is possible to have several such structures, representing multiple
	 * compression/decompression processes, in existence at once.  We refer
	 * to any one struct (and its associated working data) as a "JPEG object".
	 */
	struct jpeg_compress_struct cinfo;
	/* This struct represents a JPEG error handler.  It is declared separately
	 * because applications often want to supply a specialized error handler
	 * (see the second half of this file for an example).  But here we just
	 * take the easy way out and use the standard error handler, which will
	 * print a message on stderr and call exit() if compression fails.
	 * Note that this struct must live as long as the main JPEG parameter
	 * struct, to avoid dangling-pointer problems.
	 */
	struct my_error_mgr jerr;
	/* More stuff */
	JSAMPROW row_pointer[1];	/* pointer to JSAMPLE row[s] */
	int row_stride;		/* physical row width in image buffer */

	/* Step 1: allocate and initialize JPEG compression object */

	/* We have to set up the error handler first, in case the initialization
	 * step fails.  (Unlikely, but it could happen if you are out of memory.)
	 * This routine fills in the contents of struct jerr, and returns jerr's
	 * address which we place into the link field in cinfo.
	 */
	cinfo.err = jpeg_std_error(&jerr.pub);
	jerr.pub.error_exit = my_error_exit;
	/* Establish the setjmp return context for my_error_exit to use. */
	if (setjmp(jerr.setjmp_buffer)) {
		/* If we get here, the JPEG code has signaled an error.
		 * We need to clean up the JPEG object, close the input file, and return.
		 */
		jpeg_destroy_compress(&cinfo);
		return 0;
	}
	/* Now we can initialize the JPEG compression object. */
	jpeg_create_compress(&cinfo);

	/* Step 2: specify data destination (eg, a file) */
	/* Note: steps 2 and 3 can be done in either order. */

	jpeg_mem_dest(&cinfo, this);

	/* Step 3: set parameters for compression */

	/* First we supply a description of the input image.
	 * Four fields of the cinfo struct must be filled in:
	 */
	cinfo.image_width = width; 	/* image width and height, in pixels */
	cinfo.image_height = height;
	unsigned int csize = cinfo.image_width*cinfo.image_height;
	unsigned char * src = NULL;

	cinfo.input_components = 3;
	cinfo.in_color_space = JCS_RGB;
	row_stride = cinfo.image_width * 3;	/* JSAMPLEs per row in image_buffer */
	csize *= 3;
	src = isrc;

			
	/* Now use the library's routine to set default compression parameters.
	 * (You must set at least cinfo.in_color_space before calling this,
	 * since the defaults depend on the source color space.)
	 */
	jpeg_set_defaults(&cinfo);
	/* Now you can set any non-default parameters you wish to.
	 * Here we just illustrate the use of quality (quantization table) scaling:
	 */
	jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);

	switch (outputColorSpace) {
		case cmGray:
			cinfo.num_components = 1;
			cinfo.jpeg_color_space = JCS_GRAYSCALE;
			break;
		case cmRGB:
			cinfo.num_components = 3;
			cinfo.jpeg_color_space = JCS_RGB;
			break;
		case cmYUV:
			cinfo.num_components = 3;
			cinfo.jpeg_color_space = JCS_YCbCr;
			break;
		case cmAuto:
			break;
		default :
			fprintf(stderr,"Invalid output color type\n");
			return false;
	}

	/* Step 4: Start compressor */

	/* TRUE ensures that we will write a complete interchange-JPEG file.
	 * Pass TRUE unless you are very sure of what you're doing.
	 */
	jpeg_start_compress(&cinfo, TRUE);

	/* Step 5: while (scan lines remain to be written) */
	/*           jpeg_write_scanlines(...); */

	/* Here we use the library's state variable cinfo.next_scanline as the
	 * loop counter, so that we don't have to keep track ourselves.
	 * To keep things simple, we pass one scanline per call; you can pass
	 * more if you wish, though.
	 */

	while (cinfo.next_scanline < cinfo.image_height) {
		/* jpeg_write_scanlines expects an array of pointers to scanlines.
		 * Here the array is only one element long, but you could pass
		 * more than one scanline at a time if that's more convenient.
		 */
		row_pointer[0] = src + (cinfo.image_height - cinfo.next_scanline - 1)*row_stride;
		(void) jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}


	/* Step 6: Finish compression */

	jpeg_finish_compress(&cinfo);
	resizebuffer(((my_dest_ptr)(cinfo.dest))->bufferpos);

	/* Step 7: release JPEG compression object */

	/* This is an important step since it will release a good deal of memory. */
	jpeg_destroy_compress(&cinfo);

	return true;

}

bool JpegCodec::encode(unsigned char * src, 
		unsigned int width,
		unsigned int height, 
		const char * filename)
{
	bool res = encode(src,width,height);
	if (!res) return false;
	FILE *fp = fopen(filename,"wb");
	if (fp == NULL) {
		fprintf(stderr,"JpegCodec::encode: could not create '%s'\n",filename);
		return false;
	}
	fwrite(buffer,size,1,fp);
	fclose(fp);
	return true;
}


void JpegCodec::setOutputColorSpace(ColorSpace cspace)
{
	outputColorSpace = cspace;
}

void JpegCodec::setInputColorSpace(ColorSpace cspace)
{
	inputColorSpace = cspace;
}

bool JpegCodec::resizebuffer(unsigned int newsize)
{
	size = newsize;
	buffer = (unsigned char*)realloc(buffer,size);
	return (buffer != NULL);
}

