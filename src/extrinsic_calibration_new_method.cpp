#include "extrinsic_calibration_new_method.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ic.run();
  return 0;
}
