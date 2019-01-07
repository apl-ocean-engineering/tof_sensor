#include "pointcloud_image_fusion.h"

int main(int argc, char** argv){
  //init ROS
  ros::init(argc, argv, "TOF_Node", ros::init_options::NoSigintHandler);
  ImageFusion imageFusion;
  ros::NodeHandle n;

  //Grab ROS params
  std::string camera_info;
  if (n.getParam("/color_pointcloud/image_info", camera_info))
  {
    ROS_INFO("Camera info topic subscribed: %s", camera_info.c_str());
  }
  else{
    ROS_ERROR("No info topic specified, subscribing to /camera/rgb/camera_info");
    camera_info = "/camera/rgb/image_color";
  }

  std::string camera;
  if (n.getParam("/color_pointcloud/image", camera))
  {
    ROS_INFO("RGB Camera topic subscribed: %s", camera.c_str());
  }
  else{
    ROS_ERROR("No image topic specified, subscribing to  /camera/rgb/image_color");
    camera = "/camera/rgb/image_color";
  }

  std::string pointcloud;
  if (n.getParam("/color_pointcloud/points", pointcloud))
  {
    ROS_INFO("RGB pointcloud topic subscribed: %s", pointcloud.c_str());
  }
  else{
    ROS_ERROR("No pointcloud topic specified, subscribed to /camera/depth/points");
    pointcloud = "/camera/depth/points";
  }

  //Subscribe and spin
  ros::Subscriber image_sub = n.subscribe(camera, 1000, &ImageFusion::img_callback, &imageFusion);
  ros::Subscriber pointcloud_sub = n.subscribe(pointcloud, 1000, &ImageFusion::pointcloud_callback, &imageFusion);
  ros::Subscriber info_sub = n.subscribe(camera_info, 1000, &ImageFusion::img_info_callback, &imageFusion);

  ros::spin();
  return 0;
}
