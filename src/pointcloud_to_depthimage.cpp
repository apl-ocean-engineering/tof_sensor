#include "pointcloud_to_depthimage.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "pointcloud_to_depthimage");
  PointCloudToDepthImage DI;

  DI.run();
  //ros::spin();

  return 0;
}
