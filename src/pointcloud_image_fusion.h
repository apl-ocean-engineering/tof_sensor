/*
 * Copyright (c) 2014 Texas Instruments Inc.
 */

 /*
 Voxel ROS Node, derived from TI voxel-sdk Test folder

 12/21/2018
 author: mitchell scott
 */
#include "tof_sensor/ColorPointCloud.h"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Float64.h"

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <fstream>
#include <string>
#include <typeinfo>
#include <vector>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> colorPointCloud;

class Kinect{
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(4, 307200);
  float w = 1.0;
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat dImg;
  Eigen::Matrix4d G = Eigen::Matrix4d::Zero(4, 4);
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(3, 4);
  Eigen::Vector3d x_tilda;
  Eigen::MatrixXd points3D;
  int size;
  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  std_msgs::Float64 fx;
  std_msgs::Float64 fy;
  std_msgs::Float64 s;
  std_msgs::Float64 cx;
  std_msgs::Float64 cy;
  int imsizeX = 640;
  int imsizeY = 480;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  //tof_sensor::ColorPointCloud colorPC;
  //sensor_msgs::PointCloud pointcloud;

  ros::NodeHandle n;
  //ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("Kinect_pointcloud", 1);
  //ros::Publisher color_pointcloud_pub = n.advertise<tof_sensor::ColorPointCloud>("color_pointcloud", 1, true);
  ros::Subscriber infoSub = n.subscribe("/camera/rgb/camera_info", 1000, &Kinect::imgInfoCallback, this);
  ros::Subscriber sub = n.subscribe("/camera/rgb/image_color", 1000, &Kinect::imgCallback, this);
  ros::Subscriber kinect_pointcloud_sub = n.subscribe("/camera/depth/points", 1000, &Kinect::kinectPointCloudCallback, this);
  //ros::Publisher kibnectPub = n.advertise<colorPointCloud> ("kinect_points2", 1);
  ros::Publisher pub = n.advertise<colorPointCloud> ("color_pointcloud", 1);

  public:
    Kinect();
    int run();
    void imgInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info);
    void imgCallback(const sensor_msgs::Image::ConstPtr& img);
    void kinectPointCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
    //void processing();
};

Kinect::Kinect(){
  R = Eigen::Matrix3d::Identity(3,3);
  T = Eigen::Vector3d(0.0,0.0,0.0);
  G(0,0) = R(0,0);
  G(0,1) = R(0,1);
  G(0,2) = R(0,2);
  G(0,3) = T(0);
  G(1,0) = R(1,0);
  G(1,1) = R(1,1);
  G(1,2) = R(1,2);
  G(1,3) = T(1);
  G(2,0) = R(2,0);
  G(2,1) = R(2,1);
  G(2,2) = R(2,2);
  G(2,3) = T(2);

  G(3,3) = 1.0;
}

void Kinect::kinectPointCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

   pcl::PCLPointCloud2 pcl_pc2;
   pcl_conversions::toPCL(*input,pcl_pc2);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

   std::vector<geometry_msgs::Point32> points;
   std::vector<float> intensityData;
   std::vector<float> colorData;
   size = cloud->size();
   float w = 1.0;

   colorPointCloud::Ptr CPC (new colorPointCloud);

   CPC->header.frame_id = "map";
   CPC->points.resize(size);

   if(size>0){
     for (int i=0; i<size; i++){
       PointT kinectPoint = cloud->at(i);
       geometry_msgs::Point32 point;
       point.x = kinectPoint.x;
       point.y = kinectPoint.y;
       point.z = kinectPoint.z;
       Eigen::Vector4d x_n(point.x,point.y,point.z,w);
       X.col(i) = x_n;

       //points.push_back(point);
       //intensityData.push_back(1.0);
     }
   }
   Eigen::MatrixXd X_tilda = K*G*X;
   //std::vector<std::uint8_t> r;
   //std::vector<std::uint8_t> g;
   //std::vector<std::uint8_t> b;
   for (int i = 0; i<X_tilda.cols(); i++){
     CPC->points[i].x = X(0, i);
     CPC->points[i].y = X(1, i);
     CPC->points[i].z = X(2, i);

     int x_px = (int) X_tilda(0, i)/X_tilda(2,i);
     int y_px = (int) X_tilda(1, i)/X_tilda(2,i);
     if (x_px > imsizeX - 1){
       x_px = imsizeX -1;
     }
     else if (x_px <= 0){
       x_px = 1;
     }
     if (y_px > imsizeY - 1){
       y_px = imsizeY -1;
     }
     else if (y_px <= 0){
       y_px = 1;
     }
     if (dImg.rows > 0 && dImg.cols >0){
       cv::Vec3b color = dImg.at<cv::Vec3b>(y_px,x_px);
       //r.push_back(color(2));
       //g.push_back(color(1));
       //b.push_back(color(0));

       CPC->points[i].r = color(2);
       CPC->points[i].g = color(1);
       CPC->points[i].b = color(0);

       //Eigen::Vector4d x_n = X.col(i);
       //float x = x_n(0);
       //float y = x_n(1);
       //float z = x_n(2);
     }
   }
   pcl_conversions::toPCL(ros::Time::now(), CPC->header.stamp);
   pub.publish (CPC);
 }

void Kinect::imgCallback(const sensor_msgs::Image::ConstPtr& img){
  //Get image from Image Source
  try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      dImg =  cv_ptr->image;
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  cv::imshow("img", cv_ptr->image);
  //cv::waitKey(3);
}

void Kinect::imgInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info){
  //Get camera information
  //ROS_INFO("INFO");
  fx.data = info->K[0];
  s.data = info->K[1];
  fy.data = info->K[4];
  cx.data = info->K[2];
  cy.data = info->K[5];
  //Need to be better at Eigen.........
  K(0,0) = fx.data;
  K(0,1) = s.data;
  K(0,2) = cx.data;
  K(1,0) = 0.0;
  K(1,1) = fy.data;
  K(1,2) = cy.data;
  K(2,0) = 0.0;
  K(2,1) = 0.0;
  K(2,2) = 1.0;
}

/*
int Kinect::run(){
  //Verify ROS is working properly
  ros::Rate loop_rate(10);
  while (ros::ok()){

    loop_rate.sleep();
    ros::spinOnce();

  }

  return 0;
}
*/
