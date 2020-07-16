#ifndef CAMIMG_LOGGER_REALSENSE_IMG_LOGGER_LIB_H
#define CAMIMG_LOGGER_REALSENSE_IMG_LOGGER_LIB_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace ros;
using namespace cv;

class RealSenseImgLogger
{
public:
  RealSenseImgLogger(const ConfigParam& cfg);
  ~RealSenseImgLogger();

  void MainLoop(double dt);

private:
  ConfigParam cfgParam_;
};

#endif