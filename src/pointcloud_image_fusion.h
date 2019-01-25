/*
 * Copyright (c) 2014 Texas Instruments Inc.
 */

 /*
 Voxel ROS Node, derived from TI voxel-sdk Test folder

 12/21/2018
 author: mitchell scott
 */

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Float64.h"
#include <tf/transform_listener.h>

#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>
#include "tf_conversions/tf_eigen.h"
#include "tf/transform_datatypes.h"
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include "Eigen/Core"
#include "Eigen/Geometry"
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
  std_msgs::Float64 fx;
  std_msgs::Float64 fy;
  std_msgs::Float64 s;
  std_msgs::Float64 cx;
  std_msgs::Float64 cy;
  const int imsizeX = 640;
  const int imsizeY = 480;

  //Egien variables
  Eigen::Matrix4d G;
  Eigen::MatrixXd K;
  Eigen::Matrix3d R;
  Eigen::Vector3d T;

  //ROS Publishers and Subscribers
  tf::TransformListener listener;
  ros::NodeHandle n;
  sensor_msgs::Image image_;
  ros::Publisher color_pub = n.advertise<colorPointCloud> ("image_fusion/color_pointcloud", 1);
  ros::Publisher image_pub_ = n.advertise<sensor_msgs::Image> ("projected_image", 30);

  public:
    ImageFusion();
    void img_info_callback(const sensor_msgs::CameraInfo::ConstPtr& info);
    void img_callback(const sensor_msgs::Image::ConstPtr& img);
    void pointcloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
    void transform();
};

ImageFusion::ImageFusion(){
  /*********************************
  /Constructor. Initalize G MatrixXd
  *********************************/
  R = Eigen::Matrix3d::Identity(3,3);
  T = Eigen::Vector3d(-.14, -.99, 1.5);
  //T = Eigen::Vector3d(0, 0, 0);
  G = Eigen::Matrix4d::Zero(4, 4);
  K = Eigen::MatrixXd::Zero(3, 4);

  //Setup R and G matricies
  R(0,0) = 0.992;
  R(0,1) = 0.10;
  R(0,2) = -0.07;
  R(1,0) = -0.11;
  R(1,1) = 0.9887;
  R(1,2) = -0.1;
  R(2,0) = 0.062;
  R(2,1) = 0.11;
  R(2,2) = 0.992;

  G(0,0) = R(0,0);
  G(0,1) = R(0,1);
  G(0,2) = R(0,2);
  G(0,3) = T(0)/1000.0;
  G(1,0) = R(1,0);
  G(1,1) = R(1,1);
  G(1,2) = R(1,2);
  G(1,3) = T(1)/1000.0;
  G(2,0) = R(2,0);
  G(2,1) = R(2,1);
  G(2,2) = R(2,2);
  G(2,3) = T(2)/1000.0;
  G(3,3) = 1.0;

}

void ImageFusion::transform(){
  /**********************************************
  ROS function. Get transformation between frames
  **********************************************/
  tf::StampedTransform transform;
   try{
     listener.lookupTransform("/map", "/camera_link",
                              ros::Time(0), transform);
   }
   catch (tf::TransformException &ex) {
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
   }

   T(0) = transform.getOrigin().x();
   T(1) = transform.getOrigin().y();
   T(2) = transform.getOrigin().z();
   tf::Quaternion quaternion = transform.getRotation();
   tf::Matrix3x3 rotation_matrix(quaternion);
   tf::matrixTFToEigen(rotation_matrix, R);
   /*
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
   */
}

void ImageFusion::pointcloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){
   /**************************************************************************
   ROS callback. Convert ROS pointcloud to PCL pointcloud. Fuse with image and
   publish color pointcloud
   **************************************************************************/
   Eigen::MatrixXd X = Eigen::MatrixXd::Zero(4, size);
   const float w = 1.0;

   transform(); //Get frame transformaion

   //Convert ROS pointcloud to PCL
   pcl::PCLPointCloud2 pcl_pc2;
   pcl_conversions::toPCL(*input,pcl_pc2);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
   size = cloud->size();

   //Create new colorpointcloud message type
   colorPointCloud::Ptr CPC (new colorPointCloud);
   CPC->header.frame_id = "map";
   CPC->points.resize(size);

   //Loop through all points in pointcloud to create large homogeneous 'X' matrix
   for (int i=0; i<size; i++){
     PointT imagePoint = cloud->at(i);
     geometry_msgs::Point32 point;
     Eigen::Vector4d x_n(imagePoint.x,imagePoint.y,imagePoint.z,w);
     X.col(i) = x_n;
   }

   //Calculate image-plane points
   Eigen::MatrixXd X_tilda = K*G.inverse()*X;

   //Loop through image points and populate color pointcloud
   for (int i = 0; i<X_tilda.cols(); i++){
     //Determine location in pixels
     int x_px = (int) X_tilda(0, i)/X_tilda(2,i);
     int y_px = (int) X_tilda(1, i)/X_tilda(2,i);

     //Verify point exists within image
     if (x_px < imsizeX && x_px >= 0 && y_px < imsizeY && y_px >= 0){
       if (d_img.rows > 0 && d_img.cols >0){
         //Get RGB color from pixel location
         cv::Vec3b color = d_img.at<cv::Vec3b>(y_px,x_px);
         //Add 3D point and color information to new color pointcloud
         //For some reason, the kinect organizes as such:
         //x = z, y = -x, z = -y
         //CPC->points[i].x = X(2, i);
         //CPC->points[i].y = -X(0, i);
         //CPC->points[i].z = -X(1, i);
         CPC->points[i].x = X(0, i);
         CPC->points[i].y = X(1, i);
         CPC->points[i].z = X(2, i);
         CPC->points[i].r = color(2);
         CPC->points[i].g = color(1);
         CPC->points[i].b = color(0);
       }
     }
   }

   //Publish color pointcloud
   pcl_conversions::toPCL(ros::Time::now(), CPC->header.stamp);
   color_pub.publish (CPC);

 }

void ImageFusion::img_callback(const sensor_msgs::Image::ConstPtr& img){
  /***********************************************
  ROS callback. Turn ROS image message into cv Mat
  ***********************************************/
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
  /***********************************
  ROS callback. Get camera information
  ***********************************/
  fx.data = info->K[0];
  s.data = info->K[1];
  fy.data = info->K[4];
  cx.data = info->K[2];
  cy.data = info->K[5];
  //Population intrinsic matrix
  K(0,0) = fx.data;
  K(0,1) = s.data;
  K(0,2) = cx.data;
  K(1,1) = fy.data;
  K(1,2) = cy.data;
  K(2,2) = 1.0;
}
