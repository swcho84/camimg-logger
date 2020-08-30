#ifndef CAMIMG_LOGGER_REALSENSE_IMG_LOGGER_LIB_H
#define CAMIMG_LOGGER_REALSENSE_IMG_LOGGER_LIB_H

#include "global_header.h"
#include "config_param.h"

using namespace std;
using namespace ros;
using namespace cv;
using namespace Eigen;

class RealSenseImgLogger
{
public:
  RealSenseImgLogger(const ConfigParam& cfg);
  ~RealSenseImgLogger();

  void MainLoop(double dt);
  bool GenLogFolder(string strFolderPath, string strAhrsImgFolderPath);
  bool GenLogFile(string strFilePath);

  bool bLogFolderOpenStatus;
  bool bLogFileOpenStatus;

private:
  ConfigParam cfgParam_;
  AHRSinfo ahrsInfo_;
  AHRSinfo ahrsInfoPrev_;

  Publisher pubAhrsData_;

  sensor_msgs::CameraInfo camInfoRaw_;

  bool SaveRawImg(double dt, Mat imgInput, string strFolderPath);

  Mat GenNormDepthImg(Mat imgInput);
  Mat GenFalseColorDepthImg(Mat imgInput);
  AHRSinfo GenImuData(const sensor_msgs::ImuConstPtr& msgGyroData, const sensor_msgs::ImuConstPtr& msgAccData,
                      const sensor_msgs::ImuConstPtr& msgAttData);
  void CbSyncData(const sensor_msgs::ImageConstPtr& msgImgColorRect,
                  const sensor_msgs::ImageConstPtr& msgImgDepthAligned,
                  const sensor_msgs::CameraInfoConstPtr& msgCamInfo, const sensor_msgs::ImuConstPtr& msgGyroData,
                  const sensor_msgs::ImuConstPtr& msgAccData, const sensor_msgs::ImuConstPtr& msgAttData);

  Vector3d CalcYPREulerAngFromQuaternion(Quaterniond q);
  double wrapD(double angle);

  string GenLogColNameInfo();
  bool WritingData();
  string GenLogColTypeInfo();
  void LoggingStreamState(double dt, Mat imgInput);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;

  // ROS message for images
  cv_bridge::CvImagePtr cvPtrImgColorSrc_;
  cv_bridge::CvImagePtr cvPtrImgDepthSrc_;
  image_transport::Publisher pubFakeUsbImgRaw_;

  // synced subscriber
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image> > subColorRectImg_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image> > subDepthAlignedImg_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > subCamInfo_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Imu> > subGyroData_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Imu> > subAccData_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::Imu> > subAttData_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::CameraInfo, sensor_msgs::Imu, sensor_msgs::Imu,
                                                          sensor_msgs::Imu>
      mySyncPolicy;
  typedef message_filters::Synchronizer<mySyncPolicy> Sync;
  std::shared_ptr<Sync> sync_;

  Subscriber subXycarState_;
  void CbXycarState(const vesc_msgs::VescStateStampedConstPtr& msgVescStateStampedRaw);
  vesc_msgs::VescStateStamped msgVescStateStamped_;

  int nHeight_;
  int nWidth_;

  bool bStartCamCallBack_;

  double dAccumTime_;
  double dAhrsLogCount_;

  int nSaveCounter_;

  FILE* logFp_;

  Mat imgColorRaw_;
  Mat imgFakeUSBPub_;
  Mat imgColorLog_;
  Mat imgDepthNorm_;
  Mat imgDepthFalseColor_;

  TimeDB timeInfo_;

  double dBattVolt_;
};

#endif