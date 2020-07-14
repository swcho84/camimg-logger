#ifndef CAMIMG_LOGGER_CAMIMG_LOGGER_LIB_H
#define CAMIMG_LOGGER_CAMIMG_LOGGER_LIB_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace ros;
using namespace cv;

class CamImgLogger
{
public:
  CamImgLogger(const ConfigParam& cfg);
  ~CamImgLogger();

  void MainLoop();

private:
  ConfigParam cfgParam_;
  
};

#endif