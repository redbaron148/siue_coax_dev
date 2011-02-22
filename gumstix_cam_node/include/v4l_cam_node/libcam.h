/*
 * Copyright (C) 2009 Giacomo Spigler
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
 */

#ifndef __LIBCAM__H__
#define __LIBCAM__H__


#define USE_OPENCV 1

#ifdef USE_OPENCV
#include <opencv/cv.h>
#endif



struct buffer {
        void *                  start;
        size_t                  length;
};


typedef enum {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
} io_method;





class Camera {
private:
  void Open();
  void Close();

  void Init();
  void UnInit();

  void Start();
  void Stop();

  void init_userp(unsigned int buffer_size);
  void init_mmap();
  void init_read(unsigned int buffer_size);




public:
  const char *name;  //dev_name
  int width;
  int height;
  int fps;

  int w2;


  unsigned char *data;


  io_method io;
  int fd;
  buffer *buffers;
  int n_buffers;

  int mb, Mb, db, mc, Mc, dc, ms, Ms, ds, mh, Mh, dh, msh, Msh, dsh;
  bool ha, awb;


  Camera(const char *name, int w, int h, int fps=30);
  ~Camera();

  unsigned char *Get();    //deprecated
  unsigned char *Update(unsigned int t=100); //better  (t=0.1ms, in usecs)
  unsigned char *Update(Camera *c2, unsigned int t=100);

#ifdef USE_OPENCV
  void toIplImage(IplImage *im);
#endif




  int minBrightness();
  int maxBrightness();
  int defaultBrightness();
  int minContrast();
  int maxContrast();
  int defaultContrast();
  int minSaturation();
  int maxSaturation();
  int defaultSaturation();
  int minHue();
  int maxHue();
  int defaultHue();
  bool isHueAuto();
  int minSharpness();
  int maxSharpness();
  int defaultSharpness();
  
  //!Added by Aaron Parker
  bool isAutoWhiteBalance();
  /*int maxGamma();
  int minGamma();
  int defaultGamma();
  int maxGain();
  int minGain();
  int defaultGain();
  int maxPowerLineFreq();
  int minPowerLineFreq();
  int defaultPowerLineFreq();
  int maxWhiteBalanceTemp();
  int minWhiteBalanceTemp();
  int defaultWhiteBalanceTemp();
  int maxExposureAuto();
  int minExposureAuto();
  int defaultExposureAuto();*/

  int setBrightness(int v);
  int setContrast(int v);
  int setSaturation(int v);
  int setHue(int v);
  int setHueAuto(bool v);
  int setSharpness(int v);
  
  //! Added by Aaron Parker
  int setAutoWhiteBalance(bool v);
};





#endif
