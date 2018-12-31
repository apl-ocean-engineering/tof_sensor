/*
 * Copyright (c) 2014 Texas Instruments Inc.
 */

 /*
 Voxel ROS Node, derived from TI voxel-sdk Test folder

 12/21/2018
 author: mitchell scott
 */

#include "CameraSystem.h"
#include "Common.h"
#include "UVCStreamer.h"
#include <iomanip>
#include <fstream>
#include <string>
#include <typeinfo>
#include <vector>

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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace Voxel;

class TOF{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat dImg;
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

  pcl::PointCloud<pcl::PointXYZ> cloud;
  ros::NodeHandle n;
  ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("TOF_pointcloud", 1000);
  ros::Publisher color_pointcloud_pub = n.advertise<tof_sensor::ColorPointCloud>("color_TOF_pointcloud", 1000);
  ros::Subscriber infoSub = n.subscribe("/camera/rgb/camera_info", 1000, &TOF::imgInfoCallback, this);
  ros::Subscriber sub = n.subscribe("/camera/rgb/image_color", 1000, &TOF::imgCallback, this);
  //bool kinectPointcloud = false;
  //Kinect pointcloud subscriber... TYPICALLY COMMENT THIS OUT
  ros::Subscriber kinect_pointcloud_sub = n.subscribe("/camera/depth/points", 1000, &TOF::kinectPointCloudCallback, this);
  bool kinectPointcloud = true;

  public:
    TOF();
    int run();
    void imgInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info);
    void imgCallback(const sensor_msgs::Image::ConstPtr& img);
    void kinectPointCloudCallback(const sensor_msgs::PointCloud2 input);
    void processing(const XYZIPointCloudFrame *data);
};

TOF::TOF(){
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
  G(3,0) = R(3,0);
  G(3,1) = R(3,1);
  G(3,2) = R(3,2);
  G(3,3) = T(3);
  G(3,3) = 1.0;
}

void TOF::kinectPointCloudCallback(const sensor_msgs::PointCloud2 input)
{
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(input, pcl_pc);

  pcl::fromPCLPointCloud2(pcl_pc, cloud);
  //pcl::YOUR_PCL_FUNCTION(cloud,...);
}

void TOF::imgCallback(const sensor_msgs::Image::ConstPtr& img){
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

void TOF::imgInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& info){
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

void TOF::processing(const XYZIPointCloudFrame *data){
  //ROS pointcloud message
  //Voxel::TimeStampType lastTimeStamp = 0;
  //Voxel::TimeStampType lastTimeStamp = data->timestamp;
  tof_sensor::ColorPointCloud colorPC;
  sensor_msgs::PointCloud pointcloud;
  std_msgs::Header header;
  header.frame_id = "map";
  pointcloud.header = header;
  colorPC.header = header;
  std::vector<geometry_msgs::Point32> points;
  std::vector<float> intensityData;
  std::vector<float> colorData;
  float dataSize = data->size();
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(4, dataSize);
  if (!kinectPointcloud){
    if (dImg.rows > 0 && dImg.cols >0){
      for (int i = 0; i< dataSize; i++){
          geometry_msgs::Point32 point;
          float x = data->points.at(i).x;
          float y = data->points.at(i).y;
          float z = data->points.at(i).z;
          float w = 1.0;
          //std::cout << "x: " << x << std::endl;
          //std::cout << "y: " << y << std::endl;
          //std::cout << "z: " << z << std::endl;
          Eigen::Vector4d x_n(x,y,z, w);
          X.col(i) = x_n;

          float intensity = data->points.at(i).i;
          if (intensity > 0.0){
            point.x = x;
            point.y = y;
            point.z = z;
            points.push_back(point);
            intensityData.push_back(intensity);
          }
        }
      }
    }
  /*
  //WORKING HERE
  else{
    for (pcl::PointCloud<PointT>::iterator it = cloud.begin(); it < cloud.end(); it++)
      pcl::PointXYZ cloudPoint = *it;
      geometry_msgs::Point32 point;
      float x = cloudPoint.x;
      float y = cloudPoint.y;
      float z = cloudPoint.z;
      float w = 1.0;
  }
  */
  Eigen::MatrixXd X_tilda = K*G*X;
  //std::cout << "K: " << K << std::endl;
  //std::cout << "G: " << G << std::endl;
  std::vector<std::uint8_t> r;
  std::vector<std::uint8_t> g;
  std::vector<std::uint8_t> b;
  for (int i = 0; i<X_tilda.cols(); i++){
    //std::cout << X_tilda(0, i) << "," << X_tilda(1, i) << std::endl;
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
      r.push_back(color(2));
      g.push_back(color(1));
      b.push_back(color(0));
      //std::cout << color << std::endl;

    }
  }
  // Publish ROS info
  pointcloud.points = points;
  std::vector<sensor_msgs::ChannelFloat32> channelsData;
  //Intensity
  sensor_msgs::ChannelFloat32 intensityChannelMsg;
  intensityChannelMsg.name = "intensity";
  intensityChannelMsg.values = intensityData;
  channelsData.push_back(intensityChannelMsg);
  //publish
  pointcloud.channels = channelsData;
  pointcloud_pub.publish(pointcloud);

  colorPC.points = points;
  colorPC.r = r;
  colorPC.g = g;
  colorPC.b = b;

  color_pointcloud_pub.publish(colorPC);

}

int TOF::run(){
  //Verify ROS is working properly
  ros::Rate loop_rate(10);
  while (ros::ok()){

  	  Voxel::logger.setDefaultLogLevel(LOG_INFO);
      //Create camera objects
      CameraSystem sys;
      DepthCameraPtr depthCamera;
      //Find TI devices connected to USB
      const Vector<DevicePtr> &devices = sys.scan();
      if (devices.size() > 0)
          depthCamera = sys.connect(devices[0]); // Connect to first available device
      else
      {
          std::cerr << "SimplePCLViewer: Could not find a compatible device." << std::endl;
          return -1;
      }
      if (!depthCamera)
      {
          std::cerr << "SimplePCLViewer: Could not open a depth camera." << std::endl;
          return -1;
      }
      std::cout << "Successfully loaded depth camera for device " << depthCamera->id() << std::endl;

      //File open: dump to specified file (comment out for now...)
      /*
      char *dumpFileName = "Image.vxl";
      std::ofstream f(dumpFileName, std::ios::binary | std::ios::out);
      if (!f.good())
      {
          std::cerr << "Failed to open '" << dumpFileName << "'" << std::endl;
          return -1;
      }
      */
      int frameCount = 100;
      int count = 0;
      Voxel::TimeStampType lastTimeStamp = 0;

      std::string type = "raw";
      if (type == "raw"){
        	 depthCamera->registerCallback(DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME, [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
           const XYZIPointCloudFrame *d = dynamic_cast<const XYZIPointCloudFrame *>(&frame);
           if(!d)
           {
             std::cout << "Null frame captured? or not of type XYZIPointCloudFrame" << std::endl;
             return -1;
           }
           std::cout << "Capture frame " << d->id << "@" << d->timestamp;
           //Print FPS
           if(lastTimeStamp != 0)
             std::cout << " (" << 1E6/(d->timestamp - lastTimeStamp) << " fps)";
           std::cout << std::endl;
           lastTimeStamp = d->timestamp;
           count++;
           processing(d);
           ros::spinOnce();

        	});


	}
  if(depthCamera->start())
  {
    FrameRate r;
    if(depthCamera->getFrameRate(r))
      logger(LOG_INFO) << "Capturing at a frame rate of " << r.getFrameRate() << " fps" << std::endl;
    depthCamera->wait();
  }
  else
    logger(LOG_ERROR) << "Could not start the depth camera " << depthCamera->id() << std::endl;
}
return 0;
}
