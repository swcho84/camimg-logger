#include "config_param.h"

using namespace std;
using namespace ros;

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
  }
  catch (RosParamNotFoundException& ex)
  {
    ROS_ERROR("Failed to read param at key \"%s\"", ex.key.c_str());
    return false;
  }

  return true;
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