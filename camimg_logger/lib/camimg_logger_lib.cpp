#include "camimg_logger_lib.h"

using namespace std;
using namespace ros;
using namespace cv;

CamImgLogger::CamImgLogger(const ConfigParam& cfg) : cfgParam_(cfg)
{
}

CamImgLogger::~CamImgLogger()
{
}

// main loop
void CamImgLogger::MainLoop()
{
}