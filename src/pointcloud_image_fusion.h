/*
 * Copyright (c) 2014 Texas Instruments Inc.
 */

 /*
 Voxel ROS Node, derived from TI voxel-sdk Test folder

 12/21/2018
 author: mitchell scott
 */
//#include "tof_sensor/ColorPointCloud.h"
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

class ImageFusion{
  //Image variables
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat d_img;
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
  //Egien variables
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(4, 307200);
  Eigen::Matrix4d G = Eigen::Matrix4d::Zero(4, 4);
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(3, 4);
  //ROS Publishers and Subscribers
  ros::NodeHandle n;
  ros::Subscriber info_sub = n.subscribe("/camera/rgb/camera_info", 1000, &ImageFusion::img_info_callback, this);
  ros::Subscriber image_sub = n.subscribe("/camera/rgb/image_color", 1000, &ImageFusion::img_callback, this);
  ros::Subscriber pointcloud_sub = n.subscribe("/camera/depth/points", 1000, &ImageFusion::pointcloud_callback, this);
  ros::Publisher color_pub = n.advertise<colorPointCloud> ("color_pointcloud", 1);

  public:
    ImageFusion();
    int run();
    void img_info_callback(const sensor_msgs::CameraInfo::ConstPtr& info);
    void img_callback(const sensor_msgs::Image::ConstPtr& img);
    void pointcloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
};

ImageFusion::ImageFusion(){
  //Constructor. Initalize G MatrixXd

  //TODO make this more clean...
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

void ImageFusion::pointcloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
   //ROS callback to get Pointcloud information. Publish color pointcloud here
   //Convert from ROS pointcloud to PCL type
   pcl::PCLPointCloud2 pcl_pc2;
   pcl_conversions::toPCL(*input,pcl_pc2);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
   //Create new colorpointcloud message type
   colorPointCloud::Ptr CPC (new colorPointCloud);
   CPC->header.frame_id = "map";
   CPC->points.resize(size);
   //Loop through all points in pointcloud to create large X matrix
   size = cloud->size();
   float w = 1.0;
   for (int i=0; i<size; i++){
     PointT imagePoint = cloud->at(i);
     geometry_msgs::Point32 point;
     Eigen::Vector4d x_n(imagePoint.x,imagePoint.y,imagePoint.z,w);
     X.col(i) = x_n;
   }
   //Calculate image-plane points
   Eigen::MatrixXd X_tilda = K*G*X;
   //Loop through image points and populate color pointcloud
   for (int i = 0; i<X_tilda.cols(); i++){
     //Determine location in pixels
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
     if (d_img.rows > 0 && d_img.cols >0){
       //Get RGB color from pixel location
       cv::Vec3b color = d_img.at<cv::Vec3b>(y_px,x_px);
       //Add 3D point and color information to new color pointcloud
       CPC->points[i].x = X(0, i);
       CPC->points[i].y = X(1, i);
       CPC->points[i].z = X(2, i);
       CPC->points[i].r = color(2);
       CPC->points[i].g = color(1);
       CPC->points[i].b = color(0);
     }
   }
   //Publish color pointcloud
   pcl_conversions::toPCL(ros::Time::now(), CPC->header.stamp);
   color_pub.publish (CPC);
 }

void ImageFusion::img_callback(const sensor_msgs::Image::ConstPtr& img){
  //ROS callback to get image
  try {
    //Convert ROS image to CV image
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      d_img =  cv_ptr->image;
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
}

void ImageFusion::img_info_callback(const sensor_msgs::CameraInfo::ConstPtr& info){
  //ROS callback to get camera information
  fx.data = info->K[0];
  s.data = info->K[1];
  fy.data = info->K[4];
  cx.data = info->K[2];
  cy.data = info->K[5];
  //Population intrinsic matrix
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
