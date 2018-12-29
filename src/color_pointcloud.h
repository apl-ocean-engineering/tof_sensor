#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/CameraInfo.h"
#include <std_msgs/Float64.h>

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

static const std::string OPENCV_WINDOW = "Image window";
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class ImageColor{
  //IMSIZE: 640x480
  ros::NodeHandle n;
  PointT p;
  Eigen::Vector3d x_hat;
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat dImg;
  std::vector<geometry_msgs::Point32> _points;
  sensor_msgs::PointCloud tofPointcloud;
  Eigen::Matrix4d G = Eigen::Matrix4d::Zero(4, 4);
  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(3, 4);
  Eigen::Vector3d x_tilda;
  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  std_msgs::Float64 fx;
  std_msgs::Float64 fy;
  std_msgs::Float64 s;
  std_msgs::Float64 cx;
  std_msgs::Float64 cy;
  int imsizeX = 640;
  int imsizeY = 480;
  ros::Publisher pointcloud_pub;

  ros::Subscriber infoSub = n.subscribe("/camera/rgb/camera_info", 1000, &ImageColor::imgInfoCallback, this);
  ros::Subscriber pointcloud_sub = n.subscribe("TOF_pointcloud", 1000, &ImageColor::pointCloudCallback, this);
  ros::Subscriber sub = n.subscribe("/camera/rgb/image_color", 1000, &ImageColor::imgCallback, this);

  public:
    ImageColor(int argc, char** argv);
    void imgInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info);
    void imgCallback(const sensor_msgs::Image::ConstPtr& img);
    void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
    std::vector<PointT> processing();
    void run();
    ~ImageColor();
};

ImageColor::ImageColor(int argc, char **argv){
  T << 1,2,3;
  R = Eigen::Matrix3d::Identity(3, 3);

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
  G(3,0) = 0.0;
  G(3,1) = 0.0;
  G(3,2) = 0.0;
  G(3,3) = 1.0;
}

ImageColor::~ImageColor(){
  cv::destroyWindow(OPENCV_WINDOW);
}

void ImageColor::imgInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info){
  //Get camera information
  //ROS_INFO("INFO");
  fx.data = info->K[0];
  s.data = info->K[1];
  fy.data = info->K[4];
  cx.data = info->K[2];
  cy.data = info->K[5];
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

void ImageColor::imgCallback(const sensor_msgs::Image::ConstPtr& img){
  //Get image from Image Source
  try {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      dImg =  cv_ptr->image;
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  //cv::waitKey(3);
}

void ImageColor::pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg){
  tofPointcloud = *msg;
  _points = tofPointcloud.points;
}

std::vector<PointT> ImageColor::processing(){
  std::vector<cv::Vec3b> colorData;
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(4, _points.size());
  if (X.cols() > 0){
    for(int i = 0; i<_points.size(); i++ ) {
      geometry_msgs::Point32 point = _points.at(i);
      float w = 1.0;
      Eigen::Vector4d x_n(point.x,point.y,point.z, w);
      X.col(i) = x_n;
    }
  }
  Eigen::MatrixXd X_tilda = K*G*X;
  std::vector<PointT> pointVector;
  for (int i = 0; i<X_tilda.cols(); i++){
    geometry_msgs::Point32 point = _points.at(i);
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
      cv::Vec3b color = dImg.at<cv::Vec3b>(x_px,y_px);
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      p.r = color(0);
      p.g = color(1);
      p.b = color(2);
      pointVector.push_back(p);
    }

  }
  return pointVector;
}
