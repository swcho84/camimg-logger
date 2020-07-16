#include "realsense_img_logger_lib.h"

using namespace std;
using namespace ros;
using namespace cv;
using namespace message_filters;

RealSenseImgLogger::RealSenseImgLogger(const ConfigParam& cfg) : cfgParam_(cfg)
{
}

RealSenseImgLogger::~RealSenseImgLogger()
{
}

// main loop
void RealSenseImgLogger::MainLoop(double dt)
{
  return;
}
