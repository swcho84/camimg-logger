#include "mynteye_img_logger_lib.h"
#include "realsense_img_logger_lib.h"
#include "rotors_sim_vi_logger_lib.h"

using namespace std;
using namespace ros;
using namespace cv;

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/
int main(int argc, char** argv)
{
  // Set up ROS.
  init(argc, argv, "camera_image_logger_node");
  NodeHandle nh("");

  // reading ros params
  ConfigParam cfg;
  if (!cfg.GetRosParams())
  {
    ROS_ERROR("Wrong params!! Please check the parameter sheet..");
    return 0;
  }

  // selecting main class for image logger
  MyntEyeImgLogger myntEyeImgLogger(cfg);
  RealSenseImgLogger realSenseImgLogger(cfg);
  RotorSimViLogger rotorSimViLogger(cfg);

  // selecting main loop for image logger
  switch (cfg.nCamImgLogSensor)
  {
    case MYNTEYE:
    {
      ROS_INFO("usecase: mynteye stereo camera");

      // generating log folder
      myntEyeImgLogger.GenLogFolder(cfg.strCamImgLogFolderPath);
      break;
    }
    case REALSENSE:
    {
      ROS_INFO("usecase: realsense stereo camera");

      // generating log folder
      realSenseImgLogger.bLogFolderOpenStatus =
          realSenseImgLogger.GenLogFolder(cfg.strCamImgLogFolderPath, cfg.strAhrsImgFolderPath);
      realSenseImgLogger.bLogFileOpenStatus = realSenseImgLogger.GenLogFile(cfg.strAhrsLogFilePath);
      break;
    }
    case ROTORSVISCAM:
    {
      ROS_INFO("usecase: rotors_simulator vi camera");

      // generating log folder
      rotorSimViLogger.GenLogFolder(cfg.strCamImgLogFolderPath);
      break;
    }
    default:
    {
      ROS_INFO("Please select the correct sensor..");
      return 0;
      break;
    }
  }

  // Tell ROS how fast to run this node.
  Rate loopRate(100);

  // for calculating dt
  cfg.rosCurrTime = ros::Time::now();
  cfg.rosLastTime = ros::Time::now();

  // Main loop.
  while (ok())
  {
    // check for incoming messages
    spinOnce();

    // calculating dt
    cfg.rosCurrTime = ros::Time::now();
    cfg.dt = (cfg.rosCurrTime - cfg.rosLastTime).toSec();

    // selecting main loop for image logger
    switch (cfg.nCamImgLogSensor)
    {
      case MYNTEYE:
      {
        myntEyeImgLogger.MainLoop(cfg.dt);
        break;
      }
      case REALSENSE:
      {
        realSenseImgLogger.MainLoop(cfg.dt);
        break;
      }
      case ROTORSVISCAM:
      {
        rotorSimViLogger.MainLoop(cfg.dt);
        break;
      }
      default:
      {
        ROS_INFO("Please select the correct sensor..");
        return 0;
        break;
      }
    }

    // saving last loop time
    cfg.rosLastTime = ros::Time::now();
    loopRate.sleep();
  }

  return 0;
}  // end main()
