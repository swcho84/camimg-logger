#ifndef CAMIMG_LOGGER_ROTORS_SIM_VI_LOGGER_LIB_H
#define CAMIMG_LOGGER_ROTORS_SIM_VI_LOGGER_LIB_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace ros;
using namespace cv;

class RotorSimViLogger
{
public:
  RotorSimViLogger(const ConfigParam& cfg);
  ~RotorSimViLogger();

  void MainLoop(double dt);

private:
  ConfigParam cfgParam_;

  sensor_msgs::CameraInfo camInfoRaw_;

  bool GenLogFolder(string strFolderPath);

  void CbSyncData(const sensor_msgs::ImageConstPtr& msgImgColorRect,
                  const sensor_msgs::ImageConstPtr& msgImgDepthAligned,
                  const sensor_msgs::CameraInfoConstPtr& msgCamInfo, const nav_msgs::OdometryConstPtr& msgOdomData);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  // ROS message for images
  cv_bridge::CvImagePtr cvPtrImgColorSrc_;
  cv_bridge::CvImagePtr cvPtrImgDepthSrc_;

  // synced subscriber
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image> > subColorRectImg_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image> > subDepthAlignedImg_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > subCamInfo_;
  std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry> > subOdomData_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo, nav_msgs::Odometry>
      mySyncPolicy;
  typedef message_filters::Synchronizer<mySyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  int nHeight_;
  int nWidth_;

  bool bStartCamCallBack_;

  double dAccumTime_;
};

#endif