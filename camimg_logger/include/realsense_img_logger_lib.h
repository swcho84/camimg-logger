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
  bool GenLogFolder(string strFolderPath);

private:
  ConfigParam cfgParam_;
  BodyLinAccRotRate bodyInertialInfo_;

  sensor_msgs::CameraInfo camInfoRaw_;

  bool SaveRawImg(double dt, Mat imgInput, string strFolderPath);

  Mat GenNormDepthImg(Mat imgInput);
  Mat GenFalseColorDepthImg(Mat imgInput);
  BodyLinAccRotRate GenImuData(const sensor_msgs::ImuConstPtr& msgGyroData, const sensor_msgs::ImuConstPtr& msgAccData);

  void CbSyncData(const sensor_msgs::ImageConstPtr& msgImgColorRect,
                  const sensor_msgs::ImageConstPtr& msgImgDepthAligned,
                  const sensor_msgs::CameraInfoConstPtr& msgCamInfo, const sensor_msgs::ImuConstPtr& msgGyroData,
                  const sensor_msgs::ImuConstPtr& msgAccData);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  // ROS message for images
  cv_bridge::CvImagePtr cvPtrImgColorSrc_;
  cv_bridge::CvImagePtr cvPtrImgDepthSrc_;

  // synced subscriber
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image> > subColorRectImg_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image> > subDepthAlignedImg_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > subCamInfo_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Imu> > subGyroData_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Imu> > subAccData_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo, sensor_msgs::Imu, sensor_msgs::Imu>
      mySyncPolicy;
  typedef message_filters::Synchronizer<mySyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  int nHeight_;
  int nWidth_;

  bool bStartCamCallBack_;

  double dAccumTime_;

  Mat imgColorRaw_;
  Mat imgDepthNorm_;
  Mat imgDepthFalseColor_;
};

#endif