#include "pointcloud_image_fusion.h"

int main(int argc, char** argv){
  //init ROS
  ros::init(argc, argv, "TOF_Node", ros::init_options::NoSigintHandler);
  ImageFusion imageFusion;

  ros::spin();
  return 0;
}
