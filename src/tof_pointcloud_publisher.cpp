/*
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include "CameraSystem.h"
#include "Common.h"
#include "UVCStreamer.h"
#include <iomanip>
#include <fstream>
#include <string>
#include <typeinfo>
#include <vector>

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

using namespace Voxel;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv){
  //init ROS
  ros::init(argc, argv, "TOF_pointcloud_publisher", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::Rate loop_rate(30);
  ros::Publisher tofPub = nh.advertise<PointCloud> ("tof_pointcloud", 1);
  while (ros::ok()){
  	  Voxel::logger.setDefaultLogLevel(LOG_INFO);
      //Create camera objects
      CameraSystem sys;
      DepthCameraPtr depthCamera;
      //Find TI devices connected to USB
      const Vector<DevicePtr> &devices = sys.scan();
      if (devices.size() > 0){
          depthCamera = sys.connect(devices[0]); // Connect to first available device
      }
      else{
          std::cerr << "SimplePCLViewer: Could not find a compatible device." << std::endl;
          return -1;
      }
      if (!depthCamera){
          std::cerr << "SimplePCLViewer: Could not open a depth camera." << std::endl;
          return -1;
      }
      std::cout << "Successfully loaded depth camera for device " << depthCamera->id() << std::endl;
      int frame_count = 100;
      Voxel::TimeStampType last_time_stamp = 0;
      std::string type = "raw";
      //TOF type should be taw, if so step into callback function
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
           if(last_time_stamp != 0)
             std::cout << " (" << 1E6/(d->timestamp - last_time_stamp) << " fps)";
           std::cout << std::endl;
           //Create new pointcloud
           PointCloud::Ptr msg (new PointCloud);
           msg->header.frame_id = "map";
           //Loop through all data
           float data_size = d->size();
           msg->points.resize(data_size);
           for (int i = 0; i< data_size; i++){
             //Add points to pointcloud topic
             float intensity = d->points.at(i).i;
             if (intensity > 0.003){
               float x = d->points.at(i).x;
               float y = d->points.at(i).y;
               float z = d->points.at(i).z;
               msg->points[i].x = x;
               msg->points[i].y = y;
               msg->points[i].z = z;
             }
            }
            //Publish data
            pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
            tofPub.publish (msg);
            last_time_stamp = d->timestamp;
           ros::spinOnce();
        	});
        }
        if(depthCamera->start()){
          FrameRate r;
          if(depthCamera->getFrameRate(r))
            logger(LOG_INFO) << "Capturing at a frame rate of " << r.getFrameRate() << " fps" << std::endl;
          depthCamera->wait();
        }
        else{
          logger(LOG_ERROR) << "Could not start the depth camera " << depthCamera->id() << std::endl;
        }
      }
      return 0;
}
