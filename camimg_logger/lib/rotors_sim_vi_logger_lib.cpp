#include "rotors_sim_vi_logger_lib.h"

using namespace std;
using namespace ros;
using namespace cv;
using namespace message_filters;

RotorSimViLogger::RotorSimViLogger(const ConfigParam& cfg)
  : cfgParam_(cfg), it_(nh_), nHeight_(640), nWidth_(480), bStartCamCallBack_(false), dAccumTime_(0.0)
{
  // generating log folder
  GenLogFolder(cfgParam_.strCamImgLogFolderPath);

  // generating callback function using synced subscriber
  subColorRectImg_.reset(
      new message_filters::Subscriber<sensor_msgs::Image>(nh_, cfgParam_.strSubTpNmRotorSimViImgLeftColor, 1));
  subDepthAlignedImg_.reset(
      new message_filters::Subscriber<sensor_msgs::Image>(nh_, cfgParam_.strSubTpNmRotorSimViImgDepthRaw, 1));
  subCamInfo_.reset(
      new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, cfgParam_.strSubTpNmRotorSimViCamInfo, 1));
  subOdomData_.reset(
      new message_filters::Subscriber<nav_msgs::Odometry>(nh_, cfgParam_.strSubTpNmRotorSimViOdomData, 1));
  sync_.reset(new Sync(mySyncPolicy(cfgParam_.nRotorSimViSyncPolicy), *subColorRectImg_, *subDepthAlignedImg_, *subCamInfo_, *subOdomData_));
  sync_->registerCallback(boost::bind(&RotorSimViLogger::CbSyncData, this, _1, _2, _3, _4));
}

RotorSimViLogger::~RotorSimViLogger()
{
}

// callback function using synced data for mynteye camera
void RotorSimViLogger::CbSyncData(const sensor_msgs::ImageConstPtr& msgImgColorRect,
                                  const sensor_msgs::ImageConstPtr& msgImgDepthAligned,
                                  const sensor_msgs::CameraInfoConstPtr& msgCamInfo,
                                  const nav_msgs::OdometryConstPtr& msgOdomData)
{
}

// main loop
void RotorSimViLogger::MainLoop(double dt)
{
}

// generating log folder
bool RotorSimViLogger::GenLogFolder(string strFolderPath)
{
  // default: -1, folder generation failure status
  // 0, folder generation success
  ROS_INFO("Log folder path: %s", strFolderPath.c_str());
  boost::filesystem::path dataDir(strFolderPath.c_str());

  int nRes = -1;
  if (!boost::filesystem::is_directory(dataDir))
  {
    ROS_INFO("Making log folder in ~/.ros folder, %s", strFolderPath.c_str());

    // 0777: full permission for access, read and write
    nRes = mkdir(strFolderPath.c_str(), 0777);
  }
  else
  {
    ROS_INFO("Log folder path is exist..");
    nRes = 0;
  }

  // generating folder status
  if (nRes == 0)
  {
    ROS_INFO("Success: log folder generation, path: %s", strFolderPath.c_str());
    return true;
  }
  else if (nRes < 0)
  {
    ROS_ERROR("Failure: log folder generation, path: %s", strFolderPath.c_str());
    return false;
  }
  else
  {
    ROS_ERROR("Error: log folder generation, path: %s", strFolderPath.c_str());
    return false;
  }
}