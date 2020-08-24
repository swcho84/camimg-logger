#include "realsense_img_logger_lib.h"

using namespace std;
using namespace ros;
using namespace cv;
using namespace Eigen;
using namespace message_filters;

RealSenseImgLogger::RealSenseImgLogger(const ConfigParam& cfg)
  : cfgParam_(cfg), it_(nh_), nHeight_(640), nWidth_(480), bStartCamCallBack_(false), dAccumTime_(0.0), dAhrsLogCount_(0.0), nSaveCounter_(0)
{
  // generating callback function using synced subscriber
  subColorRectImg_.reset(
      new message_filters::Subscriber<sensor_msgs::Image>(nh_, cfgParam_.strSubTpNmRsImgColorRect, 100));
  subDepthAlignedImg_.reset(
      new message_filters::Subscriber<sensor_msgs::Image>(nh_, cfgParam_.strSubTpNmRsImgDepthAligned, 100));
  subCamInfo_.reset(new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, cfgParam_.strSubTpNmRsCamInfo, 100));
  subGyroData_.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh_, cfgParam_.strSubTpNmRsGyroDataProc, 100));
  subAccData_.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh_, cfgParam_.strSubTpNmRsAccDataProc, 100));
  subAttData_.reset(new message_filters::Subscriber<sensor_msgs::Imu>(nh_, cfgParam_.strSubTpNmRsAttDataProc, 100));
  sync_.reset(new Sync(mySyncPolicy(cfgParam_.nRsSyncPolicy), *subColorRectImg_, *subDepthAlignedImg_, *subCamInfo_,
                       *subGyroData_, *subAccData_, *subAttData_));
  sync_->registerCallback(boost::bind(&RealSenseImgLogger::CbSyncData, this, _1, _2, _3, _4, _5, _6));

  // generating publisher for the fake usb cam
  pubFakeUsbImgRaw_ = it_.advertise(cfgParam_.strPubTpNmRsFakeUsbImgRaw, 1);
}

RealSenseImgLogger::~RealSenseImgLogger()
{
}

// callback function using synced data for mynteye camera
void RealSenseImgLogger::CbSyncData(const sensor_msgs::ImageConstPtr& msgImgColorRect,
                                    const sensor_msgs::ImageConstPtr& msgImgDepthAligned,
                                    const sensor_msgs::CameraInfoConstPtr& msgCamInfo,
                                    const sensor_msgs::ImuConstPtr& msgGyroData,
                                    const sensor_msgs::ImuConstPtr& msgAccData,
                                    const sensor_msgs::ImuConstPtr& msgAttData)
{
  // grabbing the image frame, color image
  try
  {
    cvPtrImgColorSrc_ = cv_bridge::toCvCopy(msgImgColorRect, sensor_msgs::image_encodings::BGRA8);
    bStartCamCallBack_ = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("CamImgLogger::cv_bridge, color, exception: %s", e.what());
    bStartCamCallBack_ = false;
    return;
  }

  // grabbing the image frame, depth image
  try
  {
    cvPtrImgDepthSrc_ = cv_bridge::toCvCopy(msgImgDepthAligned, sensor_msgs::image_encodings::TYPE_16UC1);
    bStartCamCallBack_ = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("CamImgLogger::cv_bridge, depth exception: %s", e.what());
    bStartCamCallBack_ = false;
    return;
  }

  // making raw and depth image
  Mat& imgColorRaw = cvPtrImgColorSrc_->image;
  cvtColor(imgColorRaw, imgColorRaw_, COLOR_BGRA2BGR);
  resize(imgColorRaw_, imgColorRaw_, Size(VGAWIDTH, VGAHEIGHT), 0, 0, CV_INTER_NN);

  // for treating usecase
  imgColorRaw_.copyTo(imgFakeUSBPub_);
  imgColorRaw_.copyTo(imgColorLog_);

  // generating processed depth image
  Mat& imgDepthRaw = cvPtrImgDepthSrc_->image;
  imgDepthNorm_ = GenNormDepthImg(imgDepthRaw);
  imgDepthFalseColor_ = GenFalseColorDepthImg(imgDepthNorm_);

  // image width and height info.
  nHeight_ = imgColorRaw.rows;
  nWidth_ = imgColorRaw.cols;

  // generating inertial info w.r.t. body axis
  ahrsInfo_ = GenImuData(msgGyroData, msgAccData, msgAttData);

  // generating camera intrinsic, extrinsic information
  camInfoRaw_ = *msgCamInfo;

  // getting depth value
  // float distance = 0.001*imgDepthRaw.at<u_int16_t>(320, 240);
  // std::cout<<distance<<std::endl;
}

// main loop
void RealSenseImgLogger::MainLoop(double dt)
{
  // executing this loop only bStartCamCallBack_ is true
  if (!bStartCamCallBack_)
  {
    ROS_INFO_DELAYED_THROTTLE(10, "bStartCamCallBack_ is false..");
    return;
  }

  // saving image in the target folder
  bool bSaveRes = false;
  bSaveRes = SaveRawImg(dt, imgColorLog_, cfgParam_.strCamImgLogFolderPath);

  // saving AHRS data in the target file
  timeInfo_ = cfgParam_.GenLocalTimeVec(cfgParam_.GenLocalTimeStringFacet());
  LoggingStreamState(dt);

  // generating saved counter in fake usb raw image
  int thickness = 3;
  Point location(20, 50);
  int font = FONT_HERSHEY_SIMPLEX;
  double fontScale = 1.2;
  string strCounter;
  strCounter = "saved: " + to_string(nSaveCounter_ - 1);
  putText(imgFakeUSBPub_, strCounter, location, font, fontScale, Scalar(0, 0, 255), thickness);
  sensor_msgs::ImagePtr msgFakeUsbImgRaw = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgFakeUSBPub_).toImageMsg();
  pubFakeUsbImgRaw_.publish(msgFakeUsbImgRaw);  

  imshow("imgFakeUSBPub_", imgFakeUSBPub_);
  imshow("imgColorLog_", imgColorLog_);


  // for highgui window
  waitKey(5);
  return;
}

// saving image in the target folder
bool RealSenseImgLogger::SaveRawImg(double dt, Mat imgInput, string strFolderPath)
{
  bool bRes = false;
  string strFilePath;

  // making time accumulator and file name
  dAccumTime_ += dt;
  strFilePath = strFolderPath + "/" + "etridb_ca_raw_" + cfgParam_.GenLocalTimeStringNormal() + "." +
                cfgParam_.strCamImgLogFileType;

  // saving loop
  if (dAccumTime_ > cfgParam_.dCamImgLogHz)
  {
    imwrite(strFilePath, imgInput);
    dAccumTime_ = 0.0;
  }

  // counting saved result
  vector<String> vecPicFileNm;
  glob(strFolderPath, vecPicFileNm, true);
  nSaveCounter_ = (int)(vecPicFileNm.size());

  return bRes;
}

// generating log folder
bool RealSenseImgLogger::GenLogFolder(string strFolderPath)
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
// making imu data
AHRSinfo RealSenseImgLogger::GenImuData(const sensor_msgs::ImuConstPtr& msgGyroData,
                                        const sensor_msgs::ImuConstPtr& msgAccData,
                                        const sensor_msgs::ImuConstPtr& msgAttData)
{
  sensor_msgs::Imu GyroRaw;
  sensor_msgs::Imu AccRaw;
  sensor_msgs::Imu AttRaw;
  GyroRaw = *msgGyroData;
  AccRaw = *msgAccData;
  AttRaw = *msgAttData;

  Quaterniond quatAttRazorImu;
  Vector3d eularAttRazorImu;
  quatAttRazorImu.x() = AttRaw.orientation.x;
  quatAttRazorImu.y() = AttRaw.orientation.y;
  quatAttRazorImu.z() = AttRaw.orientation.z;
  quatAttRazorImu.w() = AttRaw.orientation.w;
  eularAttRazorImu = CalcYPREulerAngFromQuaternion(quatAttRazorImu);

  // using NED frame and 3-2-1 conversion
  AHRSinfo res;
  res.euler(0) = wrapD((eularAttRazorImu(0) - PI));
  res.euler(1) = wrapD((-1.0) * (eularAttRazorImu(1)));
  res.euler(2) = wrapD((-1.0) * (eularAttRazorImu(2) - PI));
  res.linAcc(0) = (-1.0) * (AccRaw.linear_acceleration.z);
  res.linAcc(1) = (-1.0) * (AccRaw.linear_acceleration.x);
  res.linAcc(2) = AccRaw.linear_acceleration.y;
  res.rotRate(0) = GyroRaw.angular_velocity.y;
  res.rotRate(1) = (-1.0) * (GyroRaw.angular_velocity.x);
  res.rotRate(2) = (-1.0) * (GyroRaw.angular_velocity.z);

  return res;
}

// making normalized depth image
Mat RealSenseImgLogger::GenNormDepthImg(Mat imgInput)
{
  double min;
  double max;
  minMaxIdx(imgInput, &min, &max);
  float scale = 255 / (max - min);

  // Histogram Equalization
  Mat imgRes;
  imgInput.convertTo(imgRes, CV_8UC1, scale, -min * scale);

  return imgRes;
}

// making color-based depth image
Mat RealSenseImgLogger::GenFalseColorDepthImg(Mat imgInput)
{
  // this is great. It converts your grayscale image into a tone-mapped one,
  // much more pleasing for the eye
  // function is found in contrib module, so include contrib.hpp
  // and link accordingly
  Mat imgRes;
  applyColorMap(imgInput, imgRes, COLORMAP_RAINBOW);
  return imgRes;
}

// converting the quaternion to the Euler angle(3-2-1, ZYX, YPR) [rad]
Vector3d RealSenseImgLogger::CalcYPREulerAngFromQuaternion(Quaterniond q)
{
  tf2::Quaternion quat(q.x(), q.y(), q.z(), q.w());
  tf2::Matrix3x3 matQuat(quat);
  Vector3d result;
  double dYaw, dPitch, dRoll = 0.0;
  matQuat.getEulerYPR(dYaw, dPitch, dRoll);
  result(0) = wrapD(dRoll);
  result(1) = wrapD(dPitch);
  result(2) = wrapD(dYaw);
  return result;
}

// wrap-up function, angle between -PI and PI
double RealSenseImgLogger::wrapD(double angle)
{
  angle = fmod(angle, 2.0 * PI);

  if (angle < -PI)
  {
    angle += 2.0 * PI;
  }
  else if (angle > PI)
  {
    angle -= 2.0 * PI;
  }
  else
  {
    angle = angle;
  }

  return angle;
}

// generating log file with column info
bool RealSenseImgLogger::GenLogFile(string strFilePath)
{
  // for debugging
  ROS_INFO("Log file Path:%s", strFilePath.c_str());

  // generating log file with column info
  if ((logFp_ = fopen(strFilePath.c_str(), "wb")) != NULL)
  {
    if (fprintf(logFp_, GenLogColNameInfo().c_str()) >= 0)
    {
      ROS_INFO("Log file: opened, write: success..");
      return true;
    }
    else
    {
      ROS_ERROR("Log file: opened, write: failed..");
      fclose(logFp_);
      return false;
    }
  }
  else
  {
    ROS_ERROR("Log file: opened, write: failed..");
    fclose(logFp_);
    return false;
  }
}

// generating column name info for log file
string RealSenseImgLogger::GenLogColNameInfo()
{
  string strRes;

  // column information
  ROS_INFO("Log file column setting");
  strRes = "year,month,day,hh,mm,ss,ros_time,"
           "roll,pitch,yaw,"
           "p,q,r,"
           "xbacc,ybacc,zbacc\n";

  return strRes;
}

// logging data with the sampling counter
void RealSenseImgLogger::LoggingStreamState(double dt)
{
  if (!bLogFileOpenStatus)
  {
    ROS_ERROR_DELAYED_THROTTLE(5, "Log file and write status error..please check..");
    return;
  }

  // counter, static variable
  dAhrsLogCount_ += dt;

  // writing data w.r.t the sampling hz
  if (dAhrsLogCount_ > (cfgParam_.dTimeAhrsLog))
  {
    if (!WritingData())
    {
      ROS_ERROR("Log write result: failed..");
    }

    dAhrsLogCount_ = 0.0;
  }
}

// generating column type for log file
string RealSenseImgLogger::GenLogColTypeInfo()
{
  string strRes;

  // column information
  strRes = "%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %lf, %.20Lf, "
            "%.4lf, %.4lf, %.4lf, "
            "%.4lf, %.4lf, %.4lf, "
            "%.4lf, %.4lf, %.4lf\n";
 
  return strRes;
}

// writing data w.r.t the sampling hz
bool RealSenseImgLogger::WritingData()
{
  // current: only telematry information
  // to do: with telecommand information
  bool bState = true;
  long double dRostime = (long double)(ros::Time::now().toSec());

  if (fprintf(logFp_, GenLogColTypeInfo().c_str(),
              timeInfo_.dYear,
              timeInfo_.dMonth,
              timeInfo_.dDay,
              timeInfo_.dHour,
              timeInfo_.dMin,
              timeInfo_.dSec,
              dRostime,
              ahrsInfo_.euler(0),
              ahrsInfo_.euler(1),
              ahrsInfo_.euler(2),
              ahrsInfo_.linAcc(0),
              ahrsInfo_.linAcc(1),
              ahrsInfo_.linAcc(2),
              ahrsInfo_.rotRate(0),
              ahrsInfo_.rotRate(1),
              ahrsInfo_.rotRate(2)) < 0)
  {
    ROS_ERROR("Log write, both: failed..");
    bState = false;
  }
  else
  {
    bState = true;
  }

  return bState;
}