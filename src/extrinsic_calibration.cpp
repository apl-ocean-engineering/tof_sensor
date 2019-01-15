#include "extrinsic_calibration.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  //ros::spin();
  ic.run();
  return 0;
}
