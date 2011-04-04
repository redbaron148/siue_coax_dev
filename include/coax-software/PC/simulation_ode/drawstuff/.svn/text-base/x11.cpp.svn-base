/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

// main window and event handling for X11

#include <ode/odeconfig.h>
#include "config.h"
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <GL/gl.h>
#include <GL/glew.h>
#include <GL/glxew.h>
#include <GL/glx.h>
#include <GL/glu.h>
#include <assert.h>

#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#include <drawstuff/drawstuff.h>
#include <drawstuff/version.h>
#include "internal.h"


#define CAPTURE_USES_GL
#define CAPTURE_USES_JPG

#ifdef CAPTURE_USES_JPG
#include "JpegCodec.h"
#endif

//***************************************************************************
// error handling for unix

static void printMessage (const char *msg1, const char *msg2, va_list ap)
{
	fflush (stderr);
	fflush (stdout);
	fprintf (stderr,"\n%s: ",msg1);
	vfprintf (stderr,msg2,ap);
	fprintf (stderr,"\n");
	fflush (stderr);
}


extern "C" void dsError (const char *msg, ...)
{
	va_list ap;
	va_start (ap,msg);
	printMessage ("Error",msg,ap);
	exit (1);
}


extern "C" void dsDebug (const char *msg, ...)
{
	va_list ap;
	va_start (ap,msg);
	printMessage ("INTERNAL ERROR",msg,ap);
	// *((char *)0) = 0;	 ... commit SEGVicide ?
	abort();
}


extern "C" void dsPrint (const char *msg, ...)
{
	va_list ap;
	va_start (ap,msg);
	vprintf (msg,ap);
}

//***************************************************************************
// openGL window

// X11 display info
static Display *display=0;
static int screen=0;
static XVisualInfo *visual=0;		// best visual for openGL
static Colormap colormap=0;		// window's colormap
static Atom wm_protocols_atom = 0;
static Atom wm_delete_window_atom = 0;

// window and openGL
static Window win=0;			// X11 window, 0 if not initialized
static int width=0,height=0;		// window size
static GLXContext glx_context=0;	// openGL rendering context
static int last_key_pressed=0;		// last key pressed in the window
static int run=1;			// 1 if simulation running
static int pause=0;			// 1 if in `pause' mode
static int singlestep=0;		// 1 if single step key pressed
static int writeframes=0;		// 1 if frame files to be written


static void createMainWindow (int _width, int _height)
{
	// create X11 display connection
	display = XOpenDisplay (NULL);
	if (!display) dsError ("can not open X11 display");
	screen = DefaultScreen(display);

	// get GL visual
	static int attribList[] = {GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE,16,
		GLX_RED_SIZE,4, GLX_GREEN_SIZE,4,
		GLX_BLUE_SIZE,4, None};
	visual = glXChooseVisual (display,screen,attribList);
	if (!visual) dsError ("no good X11 visual found for OpenGL");

	// create colormap
	colormap = XCreateColormap (display,RootWindow(display,screen),
			visual->visual,AllocNone);

	// initialize variables
	win = 0;
	width = _width;
	height = _height;
	glx_context = 0;
	last_key_pressed = 0;

	if (width < 1 || height < 1) dsDebug (0,"bad window width or height");

	// create the window
	XSetWindowAttributes attributes;
	attributes.background_pixel = BlackPixel(display,screen);
	attributes.colormap = colormap;
	attributes.event_mask = ButtonPressMask | ButtonReleaseMask |
		KeyPressMask | KeyReleaseMask | ButtonMotionMask | PointerMotionHintMask |
		StructureNotifyMask;
	win = XCreateWindow (display,RootWindow(display,screen),50,50,width,height,
			0,visual->depth, InputOutput,visual->visual,
			CWBackPixel | CWColormap | CWEventMask,&attributes);

	// associate a GLX context with the window
	glx_context = glXCreateContext (display,visual,0,GL_TRUE);
	if (!glx_context) dsError ("can't make an OpenGL context");

	// set the window title
	XTextProperty window_name;
	window_name.value = (unsigned char *) "Simulation";
	window_name.encoding = XA_STRING;
	window_name.format = 8;
	window_name.nitems = strlen((char *) window_name.value);
	XSetWMName (display,win,&window_name);

	// participate in the window manager 'delete yourself' protocol
	wm_protocols_atom = XInternAtom (display,"WM_PROTOCOLS",False);
	wm_delete_window_atom = XInternAtom (display,"WM_DELETE_WINDOW",False);
	if (XSetWMProtocols (display,win,&wm_delete_window_atom,1)==0)
		dsError ("XSetWMProtocols() call failed");

	// pop up the window
	XMapWindow (display,win);
	XSync (display,win);
}


static void destroyMainWindow()
{
	glXDestroyContext (display,glx_context);
	XDestroyWindow (display,win);
	XSync (display,0);
	XCloseDisplay(display);
	display = 0;
	win = 0;
	glx_context = 0;
}


static void handleEvent (XEvent &event, dsFunctions *fn)
{
	static int mx=0,my=0; 	// mouse position
	static int mode = 0;		// mouse button bits

	switch (event.type) {

		case ButtonPress: {
							  if (event.xbutton.button == Button1) mode |= 1;
							  if (event.xbutton.button == Button2) mode |= 2;
							  if (event.xbutton.button == Button3) mode |= 4;
							  mx = event.xbutton.x;
							  my = event.xbutton.y;
						  }
						  return;

		case ButtonRelease: {
								if (event.xbutton.button == Button1) mode &= (~1);
								if (event.xbutton.button == Button2) mode &= (~2);
								if (event.xbutton.button == Button3) mode &= (~4);
								mx = event.xbutton.x;
								my = event.xbutton.x;
							}
							return;

		case MotionNotify: {
							   if (event.xmotion.is_hint) {
								   Window root,child;
								   unsigned int mask;
								   XQueryPointer (display,win,&root,&child,&event.xbutton.x_root,
										   &event.xbutton.y_root,&event.xbutton.x,&event.xbutton.y,
										   &mask);
							   }
							   dsMotion (mode, event.xmotion.x - mx, event.xmotion.y - my);
							   mx = event.xmotion.x;
							   my = event.xmotion.y;
						   }
						   return;

		case KeyPress: {
						   KeySym key;
						   XLookupString (&event.xkey,NULL,0,&key,0);
						   if ((event.xkey.state & ControlMask) == 0) {
							   if (key >= ' ' && key <= 126 && fn->command) fn->command (key);
						   }
						   else if (event.xkey.state & ControlMask) {
							   switch (key) {
								   case 't': case 'T':
									   dsSetTextures (dsGetTextures() ^ 1);
									   break;
								   case 's': case 'S':
									   dsSetShadows (dsGetShadows() ^ 1);
									   break;
								   case 'x': case 'X':
									   run = 0;
									   break;
								   case 'p': case 'P':
									   pause ^= 1;
									   singlestep = 0;
									   break;
								   case 'o': case 'O':
									   if (pause) singlestep = 1;
									   break;
								   case 'v': case 'V': {
														   float xyz[3],hpr[3];
														   dsGetViewpoint (xyz,hpr);
														   printf ("Viewpoint = (%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)\n",
																   xyz[0],xyz[1],xyz[2],hpr[0],hpr[1],hpr[2]);
														   break;
													   }
								   case 'w': case 'W':
													   writeframes ^= 1;
#ifdef CAPTURE_USES_GL
													   if (writeframes) printf ("Now writing frames to TGA files\n");
#else
													   if (writeframes) printf ("Now writing frames to PPM files\n");
#endif
													   break;
							   }
						   }
						   last_key_pressed = key;		// a kludgy place to put this...
					   }
					   return;

		case KeyRelease: {
							 // hmmmm...
						 }
						 return;

		case ClientMessage:
						 if (event.xclient.message_type == wm_protocols_atom &&
								 event.xclient.format == 32 &&
								 Atom(event.xclient.data.l[0]) == wm_delete_window_atom) {
							 run = 0;
							 return;
						 }
						 return;

		case ConfigureNotify:
						 width = event.xconfigure.width;
						 height = event.xconfigure.height;
                         printf("New window size: %d %d\n",width,height);
						 return;
	}
}


// return the index of the highest bit
#ifndef CAPTURE_USES_GL
static int getHighBitIndex (unsigned int x)
{
	int i = 0;
	while (x) {
		i++;
		x >>= 1;
	}
	return i-1;
}
#endif


// shift x left by i, where i can be positive or negative
#define SHIFTL(x,i) (((i) >= 0) ? ((x) << (i)) : ((x) >> (-i)))

#ifdef CAPTURE_USES_GL

#ifdef CAPTURE_USES_JPG
JpegCodec codec(85);
#else
/// the tga header
#pragma pack(push,1)
struct TGAHeader
{
	// sometimes the tga file has a field with some custom info in. This 
	// just identifies the size of that field. If it is anything other
	// than zero, forget it.
	unsigned char m_iIdentificationFieldSize;

	// This field specifies if a colour map is present, 0-no, 1 yes...
	unsigned char m_iColourMapType;

	// only going to support RGB/RGBA/8bit - 2, colour mapped - 1
	unsigned char m_iImageTypeCode;

	// ignore this field....0
	unsigned short m_iColorMapOrigin;

	// size of the colour map
	unsigned short m_iColorMapLength;

	// bits per pixel of the colour map entries...
	unsigned char m_iColourMapEntrySize;

	// ignore this field..... 0
	unsigned short m_iX_Origin;

	// ignore this field..... 0
	unsigned short m_iY_Origin;

	// the image width....
	unsigned short m_iWidth;

	// the image height.... 
	unsigned short m_iHeight;

	// the bits per pixel of the image, 8,16,24 or 32
	unsigned char m_iBPP;

	// ignore this field.... 0
	unsigned char m_ImageDescriptorByte;
};
#pragma pack(pop)


bool WriteTga(const char* filename,
		const unsigned w,
		const unsigned h,
		const unsigned char* pixels) {

	// a flag to see if writing 32 bit image
	bool rgba=false;

	char cmd[1024];
#if 0
	sprintf(cmd,"convert tga:- jpg:%s.jpg",filename);
	FILE *fp = popen(cmd,"w");
#else
	sprintf(cmd,"%s.tga",filename);
	FILE* fp = fopen(filename,"wb");
#endif

	if(!fp) { return false; }

	// fill the file header with correct info....
	TGAHeader header;

	// wipe to 0
	memset(&header,0,sizeof(TGAHeader));

	// rgb or rgba image
	header.m_iImageTypeCode = 2;

	// set image size
	header.m_iWidth = w;
	header.m_iHeight = h;

	// set bits per pixel
	header.m_iBPP = rgba ? 32 : 24;

	// write header as first 18 bytes of output file
	fwrite(&header,sizeof(TGAHeader),1,fp);

	// get num pixels
	unsigned int total_size = w * h * 3;

	fwrite(pixels,total_size,1,fp);

#if 0
	pclose(fp);
#else
	fclose(fp);
#endif

	return true;		  
}
#endif // CAPTURE_USES_JPG

static
double now() {
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec + 1e-6*tv.tv_usec;
}

#endif

static void captureFrame (int num)
{
	static unsigned int fnum = 0;
	static double t0 = now();
	double t = now();
	if (t - t0 < 0.04) 
		return;

	t0 = t;
	if (fnum % 50 == 0) {
		fprintf (stderr,"capturing frame %04d\n",fnum);
	}


	char s[100];
	sprintf (s,"frame%03d",fnum/1000);
	mkdir(s,0755);
#ifdef CAPTURE_USES_GL
	// Warning: this assumes that the GL buffer is set to RGBA
	// see glXChooseVisual 
	unsigned int bpp = 3;
    unsigned int w,h;
	glReadBuffer(GL_FRONT);

    w = 4*(width/4);
    h = 4*(height/4);
	unsigned char* pdata = new unsigned char[(w+16)*(h+16)*bpp];
#ifdef CAPTURE_USES_JPG
	// glReadPixels(0,0,width,height,GL_RGB,GL_UNSIGNED_BYTE,pdata);
	glReadPixels(0,0,w,h,GL_RGB,GL_UNSIGNED_BYTE,pdata);
	sprintf (s,"frame%03d/frame%04d.jpg",fnum/1000,fnum%1000);
	codec.encode(pdata,w,h,s);
#else
	glReadPixels(0,0,w,h,GL_BGR,GL_UNSIGNED_BYTE,pdata);
	sprintf (s,"frame%03d/frame%04d",fnum/1000,fnum%1000);
	WriteTga(s,w,h,pdata);
#endif
	delete [] pdata;
#else
	sprintf (s,"frame%03d/frame%04d.ppm",fnum/1000,fnum%1000);
	FILE *f = fopen (s,"wb");
	if (!f) dsError ("can't open \"%s\" for writing",s);
	fprintf (f,"P6\n%d %d\n255\n",width,height);
	XImage *image = XGetImage (display,win,0,0,width,height,~0,ZPixmap);

	int rshift = 7 - getHighBitIndex (image->red_mask);
	int gshift = 7 - getHighBitIndex (image->green_mask);
	int bshift = 7 - getHighBitIndex (image->blue_mask);

	size_t size = width * height * 3;
	unsigned char *buffer=(unsigned char*)malloc(size);
	if (!buffer) dsError ("can't allocate intermediary buffer of size %d",size);
	for (int y=0; y<height; y++) {
		for (int x=0; x<width; x++) {
			unsigned long pixel = XGetPixel (image,x,y);
			unsigned char *b = buffer+3*width*y+3*x;
			b[0] = SHIFTL(pixel & image->red_mask,rshift);
			b[1] = SHIFTL(pixel & image->green_mask,gshift);
			b[2] = SHIFTL(pixel & image->blue_mask,bshift);
		}
	}
	fwrite (buffer,size,1,f);
	fclose (f);
	free(buffer);
	XDestroyImage (image);
#endif
	fnum += 1;
}


void dsPlatformSimLoop (int window_width, int window_height, dsFunctions *fn,
		int initial_pause)
{
	pause = initial_pause;
	createMainWindow (window_width, window_height);
	glXMakeCurrent (display,win,glx_context);

	dsStartGraphics (window_width,window_height,fn);

	static bool firsttime=true;
	if (firsttime)
	{
		fprintf
			(
			 stderr,
			 "\n"
			 "Simulation test environment v%d.%02d\n"
			 "   Ctrl-P : pause / unpause (or say `-pause' on command line).\n"
			 "   Ctrl-O : single step when paused.\n"
			 "   Ctrl-T : toggle textures (or say `-notex' on command line).\n"
			 "   Ctrl-S : toggle shadows (or say `-noshadow' on command line).\n"
			 "   Ctrl-V : print current viewpoint coordinates (x,y,z,h,p,r).\n"
#ifdef CAPTURE_USES_GL
			 "   Ctrl-W : write frames to TGA files: frameNNN/frameNNN.tga\n"
#else
			 "   Ctrl-W : write frames to PPM files: frameNNN/frameNNN.ppm\n"
#endif
			 "   Ctrl-X : exit.\n"
			 "\n"
			 "Change the camera position by clicking + dragging in the window.\n"
			 "   Left button - pan and tilt.\n"
			 "   Right button - forward and sideways.\n"
			 "   Left + Right button (or middle button) - sideways and up.\n"
			 "\n",DS_VERSION >> 8,DS_VERSION & 0xff
			 );
		firsttime = false;
	}

	if (fn->start) fn->start();

	int frame = 1;
	run = 1;
	while (run) {
		// read in and process all pending events for the main window
		XEvent event;
		while (run && XPending (display)) {
			XNextEvent (display,&event);
			handleEvent (event,fn);
		}

		dsDrawFrame (width,height,fn,pause && !singlestep);
		singlestep = 0;

		glFlush();
		glXSwapBuffers (display,win);
		XSync (display,0);

		// capture frames if necessary
		if (pause==0 && writeframes) {
			captureFrame (frame);
			frame++;
		}
	};

	if (fn->stop) fn->stop();
	dsStopGraphics();

	destroyMainWindow();
}


extern "C" void dsStop()
{
	run = 0;
}


extern "C" double dsElapsedTime()
{
#if HAVE_GETTIMEOFDAY
	static double prev=0.0;
	timeval tv ;

	gettimeofday(&tv, 0);
	double curr = tv.tv_sec + (double) tv.tv_usec / 1000000.0 ;
	if (!prev)
		prev=curr;
	double retval = curr-prev;
	prev=curr;
	if (retval>1.0) retval=1.0;
	if (retval<dEpsilon) retval=dEpsilon;
	return retval;
#else
	return 0.01666; // Assume 60 fps
#endif
}



