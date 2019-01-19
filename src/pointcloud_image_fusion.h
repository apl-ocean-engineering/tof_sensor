/*
 * Copyright (c) 2014 Texas Instruments Inc.
 */

 /*
 Voxel ROS Node, derived from TI voxel-sdk Test folder

 12/21/2018
 author: mitchell scott
 */
//#include "tof_sensor/ColorPointCloud.h"
#include <fstream>
#include <string>
#include <typeinfo>
#include <vector>
#include <iostream>

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Float64.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
  int imsizeX = 480;
  int imsizeY = 640;
  //Egien variables
  Eigen::Matrix4d G;
  Eigen::MatrixXd K;
  //ROS Publishers and Subscribers
  tf::TransformListener listener;
  ros::NodeHandle n;
  colorPointCloud::Ptr CPC;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;


  sensor_msgs::Image image_;
  ros::Publisher color_pub = n.advertise<colorPointCloud>
                                          ("image_fusion/color_pointcloud", 1);
  ros::Publisher image_pub_ = n.advertise<sensor_msgs::Image>
                                                       ("projected_image", 30);

  public:
    ImageFusion();
    void img_info_callback(const sensor_msgs::CameraInfo::ConstPtr& info);
    void img_callback(const sensor_msgs::Image::ConstPtr& img);
    void pointcloud_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
    void pointcloud_callback_img_pub(const sensor_msgs::PointCloud2ConstPtr& input);
    void transform();
};

ImageFusion::ImageFusion() : CPC(new colorPointCloud), cloud(new pcl::PointCloud<pcl::PointXYZ>){
  //Constructor. Initalize G MatrixXd


  R = Eigen::Matrix3d::Identity(3,3);
  /*
  R << 0.669719, 0.740817, 0.0516332,
       0.034626, 0.038301, -0.998666,
      -0.74180, 0.670614, 0;
  */
  //Eigen::Vector3d T(0.0, 0.0, 0.0);
  T << 0.0583417, -0.0856591, 0;

  G = Eigen::Matrix4d::Identity();
  K = Eigen::MatrixXd::Zero(3, 4);

  G(0,0) = 0.667;
  G(0,1) = 0.740817;
  G(0,2) = 0.0516;
  G(0,3) = T(0);

  G(1,0) = 0.034626;
  G(1,1) = 0.038301;
  G(1,2) = -0.998666;
  G(1,3) = T(1);

  G(2,0) = -0.7442;
  G(2,1) = 0.668;
  G(2,2) = -0.003;
  G(2,3) = T(2);
  //G(3,3) = 1.0;

}

void ImageFusion::pointcloud_callback_img_pub(const sensor_msgs::PointCloud2ConstPtr& input){
  ROS_INFO("here");
  sensor_msgs::Image image_;
  if ((input->width * input->height) == 0){
    return; //return if the cloud is not dense!
  }
  try {
    pcl::toROSMsg (*input, image_); //convert the cloud
  }
  catch (std::runtime_error e)
  {
    ROS_ERROR_STREAM("Error in converting cloud to image message: "
                    << e.what());
  }
  image_.header.frame_id = "map";
  image_pub_.publish (image_); //publish our cloud image
}

void ImageFusion::transform(){
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
   //ROS callback to get Pointcloud information. Publish color pointcloud here
   //Convert from ROS pointcloud to PCL type
   transform(); //Transform to frame coordinates

   pcl::PCLPointCloud2 pcl_pc2;
   pcl_conversions::toPCL(*input,pcl_pc2);

   //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

   pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
   size = cloud->size();
   Eigen::MatrixXd X = Eigen::MatrixXd::Zero(4, size);
   float w = 1.0;
   //Create new colorpointcloud message type

   CPC->header.frame_id = "map";
   CPC->points.resize(size);
   //Loop through all points in pointcloud to create large X matrix
   for (int i=0; i<size; i++){
     PointT imagePoint = cloud->at(i);
     geometry_msgs::Point32 point;
     if (imagePoint.z > 0){
       //Verify we're above z depth
       Eigen::Vector4d x_n(imagePoint.x,imagePoint.y,imagePoint.z,w);
       X.col(i) = x_n;
     }
   }
   //Calculate image-plane points
   Eigen::Matrix3d trans;
   //trans << -1,0,0,0,1,0,0,0,1;
   Eigen::MatrixXd X_tilda = K*G*X;
   //std::cout << "X_tilda size : " << X_tilda.rows() << "," << X_tilda.cols() << std::endl;
   //std::cout << K << std::endl;
   //Loop through image points and populate color pointcloud
   for (int i = 0; i<X_tilda.cols(); i++){
     //Determine location in pixels
     int x_px = (int) X_tilda(0, i)/X_tilda(2,i);
     int y_px = (int) X_tilda(1, i)/X_tilda(2,i);
     //std::cout << "G: " << G << std::endl << std::endl;
     //std::cout << "K: " << K << std::endl << std::endl;
     /*
     if (X_tilda(2, i) == 0){
       std::cout << "X" << X.col(i) << std::endl;
       std::cout << "X_tilda" << X_tilda.col(i) << std::endl;
       std::cout << G << std::endl;
     }
     */


     if (x_px < imsizeX && x_px >= 0 && y_px < imsizeY && y_px >= 0){
       if (d_img.rows > 0 && d_img.cols >0){
         //Get RGB color from pixel location
         cv::Vec3b color = d_img.at<cv::Vec3b>(x_px,y_px);
         //Add 3D point and color information to new color pointcloud
         //For some reason, the kinect organizes as such:
         //x = y, y = -x, z = z
         CPC->points[i].x = X(1, i);
         CPC->points[i].y = -X(0, i);
         CPC->points[i].z = X(2, i);
         //CPC->points[i].x = -X(1, i);
         //CPC->points[i].y = X(0, i);
         //CPC->points[i].z = X(2, i);
         CPC->points[i].r = color(2);
         CPC->points[i].g = color(1);
         CPC->points[i].b = color(0);
       }
     }
   }
   //std::cout << d_img.size() << std::endl;
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
  K(1,1) = fy.data;
  K(1,2) = cy.data;
  K(2,2) = 1.0;
}
