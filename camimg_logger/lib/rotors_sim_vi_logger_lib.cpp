#include "rotors_sim_vi_logger_lib.h"

using namespace std;
using namespace ros;
using namespace cv;
using namespace Eigen;
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
  sync_.reset(new Sync(mySyncPolicy(cfgParam_.nRotorSimViSyncPolicy), *subColorRectImg_, *subDepthAlignedImg_,
                       *subCamInfo_, *subOdomData_));
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
  // grabbing the image frame, color image
  try
  {
    cvPtrImgColorSrc_ = cv_bridge::toCvCopy(msgImgColorRect, sensor_msgs::image_encodings::BGR8);
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
    cvPtrImgDepthSrc_ = cv_bridge::toCvCopy(msgImgDepthAligned, sensor_msgs::image_encodings::BGR8);
    bStartCamCallBack_ = true;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("CamImgLogger::cv_bridge, depth exception: %s", e.what());
    bStartCamCallBack_ = false;
    return;
  }

  // generating camera intrinsic, extrinsic information
  camInfoRaw_ = *msgCamInfo;

  // making raw and depth image
  imgColorRaw_ = cvPtrImgColorSrc_->image;

  // // generating processed depth image
  imgDepthRaw_ = cvPtrImgDepthSrc_->image;

  // getting depth value, tested(ok)
  // float distance = 0.001*imgDepthRaw.at<u_int16_t>(320, 240);
  // std::cout<<distance<<std::endl;

  // generating inertial info, position
  odomData_ = *msgOdomData;
  Vector3d posNed;
  Vector3d posEnu;
  posEnu(0) = odomData_.pose.pose.position.x;
  posEnu(1) = odomData_.pose.pose.position.y;
  posEnu(2) = odomData_.pose.pose.position.z;
  posNed = ConvertPosFromEnuToNed(posEnu);

  // making Euler angle w.r.t NED frame
  Quaterniond quatAtt;
  quatAtt.x() = odomData_.pose.pose.orientation.x;
  quatAtt.y() = odomData_.pose.pose.orientation.y;
  quatAtt.z() = odomData_.pose.pose.orientation.z;
  quatAtt.w() = odomData_.pose.pose.orientation.w;
  eularAng = CalcYPREulerAngFromQuaternion(quatAtt);
}

// main loop
void RotorSimViLogger::MainLoop(double dt)
{
  // executing this loop only bStartCamCallBack_ is true
  if (!bStartCamCallBack_)
    return;

  // saving image in the target folder
  SaveRawImg(dt, imgColorRaw_, cfgParam_.strCamImgLogFolderPath);

  // for highgui window
  waitKey(5);
  return;
}

// saving image in the target folder
bool RotorSimViLogger::SaveRawImg(double dt, Mat imgInput, string strFolderPath)
{
  bool bRes = false;
  string strFilePath;

  // making time accumulator and file name
  dAccumTime_ += dt;
  strFilePath = strFolderPath + "/" + "karidb_land_raw_" + cfgParam_.GenLocalTimeStringNormal() + "." +
                cfgParam_.strCamImgLogFileType;

  // saving loop
  if (dAccumTime_ > cfgParam_.dCamImgLogHz)
  {
    imwrite(strFilePath, imgInput);
    dAccumTime_ = 0.0;
  }

  return bRes;
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

// converting the quaternion to the Euler angle(3-2-1, ZYX, YPR) [rad]
Vector3d RotorSimViLogger::CalcYPREulerAngFromQuaternion(Quaterniond q)
{
  tf2::Quaternion quat(q.x(), q.y(), q.z(), q.w());
  tf2::Matrix3x3 matQuat(quat);
  Vector3d result;
  double dYaw, dPitch, dRoll = 0.0;
  matQuat.getEulerYPR(dYaw, dPitch, dRoll);
  result(0) = wrapD(dRoll);
  result(1) = wrapD(dPitch);
  result(2) = wrapD((-1.0) * (dYaw));
  return result;
}

// converting from the position w.r.t ENU frame to the position w.r.t NED frame
Vector3d RotorSimViLogger::ConvertPosFromEnuToNed(Vector3d posEnu)
{
  // tested(ok)
  Vector3d result;
  Vector3d attForEnuToNed;
  Matrix3d dcmForEnuToNed;
  attForEnuToNed(0) = (-180.0) * (D2R);
  attForEnuToNed(1) = (0.0) * (D2R);
  attForEnuToNed(2) = (90.0) * (D2R);
  dcmForEnuToNed = CalcDcmEuler321(attForEnuToNed);
  result = (dcmForEnuToNed) * (posEnu);
  return result;
}

// calculating DCM, Euler angle, 321 conversion (ref:from NED to Body, using Euler angle (3->2->1))
// only 3-2-1 convention
// [          cy*cz,          cy*sz,            -sy]
// [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx]
// [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]
Matrix3d RotorSimViLogger::CalcDcmEuler321(Vector3d eulerAtt)
{
  Matrix3d result;
  double cx = cos(eulerAtt(0));
  double cy = cos(eulerAtt(1));
  double cz = cos(eulerAtt(2));
  double sx = sin(eulerAtt(0));
  double sy = sin(eulerAtt(1));
  double sz = sin(eulerAtt(2));
  result(0, 0) = cy * cz;
  result(0, 1) = cy * sz;
  result(0, 2) = -sy;
  result(1, 0) = sy * sx * cz - sz * cx;
  result(1, 1) = sy * sx * sz + cz * cx;
  result(1, 2) = cy * sx;
  result(2, 0) = sy * cx * cz + sz * sx;
  result(2, 1) = sy * cx * sz - cz * sx;
  result(2, 2) = cy * cx;
  return result;
}

// wrap-up function, angle between -PI and PI
double RotorSimViLogger::wrapD(double angle)
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