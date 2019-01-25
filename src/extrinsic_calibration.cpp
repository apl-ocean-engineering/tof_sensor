#include "extrinsic_calibration.h"

int main(int argc, char** argv)
{
  //Init ROS
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle n;

  //Get ROS caera info
  std::string camera_info;
  if (n.getParam("/color_pointcloud/image_info", camera_info))
  {
    ROS_INFO("Camera info topic subscribed: %s", camera_info.c_str());
  }
  else{
    ROS_ERROR("No info topic specified, subscribing to /camera/rgb/camera_info");
    camera_info = "/camera/rgb/camera_info";
  }

  std::string image_topic;
  if (n.getParam("/color_pointcloud/image", image_topic))
  {
    ROS_INFO("RGB Camera topic subscribed: %s", image_topic.c_str());
  }
  else{
    ROS_ERROR("No image topic specified, subscribing to  /camera/rgb/image_color");
    image_topic = "/camera/rgb/image_color";
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

  //Create salibration obect and subscribe to ROS topics
  ExtrinsicCalibration ec(image_topic);
  ros::Subscriber info_sub = n.subscribe(camera_info, 1000,
                              &ExtrinsicCalibration::img_info_callback, &ec);
  ros::Subscriber pointcloud_sub = n.subscribe(pointcloud, 1000,
                              &ExtrinsicCalibration::pointcloud_callback, &ec);

  //Run main function
  ec.run();

  return 0;
}
