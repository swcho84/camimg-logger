#include "config_param.h"

using namespace std;
using namespace ros;
using namespace cv;

ConfigParam::ConfigParam()
{
}

ConfigParam::~ConfigParam()
{
}

// reading rosparams (public function)
bool ConfigParam::GetRosParams()
{
  return ReadRosParams();
}

// reading rosparams (private function)
bool ConfigParam::ReadRosParams()
{
  try
  {
    NodeHandle nh("");

    // general information
    strHomeName = getenv("HOME");

    // topic name, for subscriber, mynteye stereo camera
    ReadRosParam(nh, "/MyntEyeDStereoCam/SubTopicName/CamInfo", strSubTpNmMyntEyeCamInfo);
    ReadRosParam(nh, "/MyntEyeDStereoCam/SubTopicName/ColorImg", strSubTpNmMyntEyeImgLeftColor);
    ReadRosParam(nh, "/MyntEyeDStereoCam/SubTopicName/DepthImg", strSubTpNmMyntEyeImgDepthRaw);
    ReadRosParam(nh, "/MyntEyeDStereoCam/SubTopicName/ImuData", strSubTpNmMyntEyeImuDataProc);

    // topic name, for publisher, mynteye stereo camera
    ReadRosParam(nh, "/MyntEyeDStereoCam/PubTopicName/ImgImuSync", strPubTpNmMyntEyeImgImuSync);

    // params mynteye stereo camera
    ReadRosParam(nh, "/MyntEyeDStereoCam/SyncPolicy/time", nMyntEyeSyncPolicy);

    // topic name, for subscriber, realsense stereo camera
    ReadRosParam(nh, "/RealSenseStereoCam/SubTopicName/CamInfo", strSubTpNmRsCamInfo);
    ReadRosParam(nh, "/RealSenseStereoCam/SubTopicName/ColorImg", strSubTpNmRsImgColorRect);
    ReadRosParam(nh, "/RealSenseStereoCam/SubTopicName/DepthImg", strSubTpNmRsImgDepthAligned);
    ReadRosParam(nh, "/RealSenseStereoCam/SubTopicName/GyroData", strSubTpNmRsGyroDataProc);
    ReadRosParam(nh, "/RealSenseStereoCam/SubTopicName/AccData", strSubTpNmRsAccDataProc);
    ReadRosParam(nh, "/RealSenseStereoCam/SubTopicName/AttData", strSubTpNmRsAttDataProc);

    // topic name, for publisher, realsense stereo camera
    ReadRosParam(nh, "/RealSenseStereoCam/PubTopicName/ImuData", strPubTpNmRsImuDataProc);
    ReadRosParam(nh, "/RealSenseStereoCam/PubTopicName/ImgImuSync", strPubTpNmRsImgImuSync);
    ReadRosParam(nh, "/RealSenseStereoCam/PubTopicName/FakeUsbImg", strPubTpNmRsFakeUsbImgRaw);

    // params realsense stereo camera
    ReadRosParam(nh, "/RealSenseStereoCam/SyncPolicy/time", nRsSyncPolicy);

    // topic name, for subscriber, rotors_simulator vi camera
    ReadRosParam(nh, "/RotorsSimViCam/SubTopicName/CamInfo", strSubTpNmRotorSimViCamInfo);
    ReadRosParam(nh, "/RotorsSimViCam/SubTopicName/ColorImg", strSubTpNmRotorSimViImgLeftColor);
    ReadRosParam(nh, "/RotorsSimViCam/SubTopicName/DepthImg", strSubTpNmRotorSimViImgDepthRaw);
    ReadRosParam(nh, "/RotorsSimViCam/SubTopicName/OdomData", strSubTpNmRotorSimViOdomData);

    // topic name, for publisher, rotors_simulator vi camera
    ReadRosParam(nh, "/RotorsSimViCam/PubTopicName/ImgImuSync", strPubTpNmRotorSimViImgImuSync);

    // params rotors_simulator vi camera
    ReadRosParam(nh, "/RotorsSimViCam/SyncPolicy/time", nRotorSimViSyncPolicy);

    // camera image logger folder and speed info.
    ReadRosParam(nh, "/CamImgLog/ComCase", nCamImgLogComCase);
    ReadRosParam(nh, "/CamImgLog/Sensor", nCamImgLogSensor);
    ReadRosParam(nh, "/CamImgLog/Info/Type", strCamImgLogFileType);
    ReadRosParam(nh, "/CamImgLog/Info/Name", strCamImgLogFolderNm);
    ReadRosParam(nh, "/CamImgLog/Info/Hz", dCamImgLogHz);

    // making folder path
    string strMidPath;
    switch (nCamImgLogComCase)
    {
      case MAINCOM:
      {
        strMidPath = "/.ros";
        break;
      }
      case EMBEDDED:
      {
        strMidPath = "/camimg_log";
        break;
      }
      default:
      {
        strMidPath = "/.ros";
        break;
      }
    }
    strCamImgLogFolderPath = strHomeName + strMidPath + "/" + GenLocalTimeStringNormal() + "_" + strCamImgLogFolderNm;

    int nRand = 0;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 325679);
    nRand = dis(gen);
    dTimeAhrsLog = 0.2;  // 0.2 [sec]
    strAhrsImgFolderPath = strCamImgLogFolderPath + "/ahrs_img";
    strAhrsLogFilePath = strCamImgLogFolderPath + "/" + GenLocalTimeStringNormal() + "_" + "AHRS_log" + to_string(nRand) + ".csv";
  }
  catch (RosParamNotFoundException& ex)
  {
    ROS_ERROR("Failed to read param at key \"%s\"", ex.key.c_str());
    return false;
  }

  return true;
}

// generating local time from string to double variables
TimeDB ConfigParam::GenLocalTimeVec(string strTime)
{
  std::istringstream iss(strTime);
  vector<string> wordsVector{ istream_iterator<string>{ iss }, istream_iterator<string>{} };
  double arrRes[6];
  for (auto i = 0u; i < wordsVector.size(); i++)
    arrRes[i] = stod(wordsVector[i]);

  TimeDB res;
  res.dYear = arrRes[0];
  res.dMonth = arrRes[1];
  res.dDay = arrRes[2];
  res.dHour = arrRes[3];
  res.dMin = arrRes[4];
  res.dSec = arrRes[5];

  return res;
}

// generating local time to string with facet
string ConfigParam::GenLocalTimeStringFacet()
{
  std::stringstream ss;
  boost::local_time::local_date_time currentTime(boost::posix_time::microsec_clock::local_time(),
                                                 boost::local_time::time_zone_ptr());
  auto facet = new boost::posix_time::time_facet("%Y %m %d %H %M %s");
  ss.imbue(std::locale(std::locale::classic(), facet));
  ss << currentTime.local_time();
  return ss.str();
}

// generating local time to string without facet
string ConfigParam::GenLocalTimeStringNormal()
{
  std::stringstream ss;
  boost::local_time::local_date_time currentTime(boost::posix_time::second_clock::local_time(),
                                                 boost::local_time::time_zone_ptr());
  ss << currentTime.local_time();
  return ss.str();
}

void ConfigParam::ReadRosParam(ros::NodeHandle& nh, const string& key, float& val)
{
  if (!nh.hasParam(key))
    throw RosParamNotFoundException(key);
  nh.getParam(key, val);
}

void ConfigParam::ReadRosParam(ros::NodeHandle& nh, const string& key, double& val)
{
  if (!nh.hasParam(key))
    throw RosParamNotFoundException(key);
  nh.getParam(key, val);
}

void ConfigParam::ReadRosParam(ros::NodeHandle& nh, const string& key, bool& val)
{
  if (!nh.hasParam(key))
    throw RosParamNotFoundException(key);
  nh.getParam(key, val);
}

void ConfigParam::ReadRosParam(ros::NodeHandle& nh, const string& key, int32_t& val)
{
  if (!nh.hasParam(key))
    throw RosParamNotFoundException(key);
  nh.getParam(key, val);
}

void ConfigParam::ReadRosParam(ros::NodeHandle& nh, const string& key, string& val)
{
  if (!nh.hasParam(key))
    throw RosParamNotFoundException(key);
  nh.getParam(key, val);
  if (val.empty())
    throw RosParamNotFoundException(key);
}