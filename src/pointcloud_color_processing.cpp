/*
Color Pointcloud Transformation ROS node

12/30/2018
author: mitchell scott
*/
#include "geometry_msgs/Point32.h"
#include "ros/ros.h"
#include "tof_sensor/ColorPointCloud.h"
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class colorProcessing{
  tof_sensor::ColorPointCloud CPC;
  public:
    void callback(const tof_sensor::ColorPointCloud::ConstPtr& msg);
    tof_sensor::ColorPointCloud getData();
};

void colorProcessing::callback(const tof_sensor::ColorPointCloud::ConstPtr& msg){
  CPC = *msg;
}

tof_sensor::ColorPointCloud colorProcessing::getData(){
  return CPC;
}

int main(int argc, char** argv){
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  colorProcessing colorProc;
  ros::Subscriber sub = nh.subscribe("/color_TOF_pointcloud", 1000, &colorProcessing::callback, &colorProc);

  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);
  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "map";
  std::vector<geometry_msgs::Point32> points;
  geometry_msgs::Point32 point;
  std::vector<std::uint8_t> redChannel;
  std::vector<std::uint8_t> greenChannel;
  std::vector<std::uint8_t> blueChannel;
  ros::Rate loop_rate(100);
  while (nh.ok()){
    tof_sensor::ColorPointCloud colorData = colorProc.getData();
    points = colorData.points;
    redChannel = colorData.r;
    greenChannel = colorData.g;
    blueChannel = colorData.b;

    msg->points.resize(points.size());
    for(std::size_t i=0; i<points.size(); ++i){
        point = points[i];
        float r = redChannel[i];
        float g = greenChannel[i];
        float b = blueChannel[i];
        msg->points[i].x = point.x;
        msg->points[i].y = point.y;
        msg->points[i].z = point.z;
        msg->points[i].r = r;
        msg->points[i].g = g;
        msg->points[i].b = b;
    }

    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }

  return 0;
}
