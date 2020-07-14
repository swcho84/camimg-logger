#include "camimg_logger_lib.h"

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

  // main class
  CamImgLogger camImgLogger(cfg);

  // Tell ROS how fast to run this node.
  Rate loopRate(30);

  // Main loop.
  while (ok())
  {
    camImgLogger.MainLoop();

    spinOnce();
    loopRate.sleep();
  }

  return 0;
}  // end main()
