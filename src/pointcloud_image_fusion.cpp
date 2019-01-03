#include "pointcloud_image_fusion.h"
#include "ros/ros.h"
using namespace Eigen;
int main(int argc, char** argv){
  //init ROS
  ros::init(argc, argv, "TOF_Node", ros::init_options::NoSigintHandler);
  Kinect kinect;

  ros::spin();
  //kinect.run();


  return 0;
}
