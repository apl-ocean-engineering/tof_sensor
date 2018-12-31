/*
 * Copyright (c) 2014 Texas Instruments Inc.
 */

 /*
 Voxel ROS Node, derived from TI voxel-sdk Test folder

 12/21/2018
 author: mitchell scott
 */

#include "TOF.h"

using namespace Voxel;

int main(int argc, char** argv){
  //init ROS
  ros::init(argc, argv, "TOF_Node", ros::init_options::NoSigintHandler);
  TOF tof;
  tof.run();
  return 0;
}
