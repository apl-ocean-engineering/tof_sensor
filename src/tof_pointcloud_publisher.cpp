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

#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include "image_transport/image_transport.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace Voxel;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef pcl::PointXYZI PointT;

int main(int argc, char** argv){
  //init ROS
  ros::init(argc, argv, "TOF_pointcloud_publisher",
                                          ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  ros::Rate loop_rate(30);
  ros::Publisher tofPub = nh.advertise<PointCloud> ("tof/points/raw", 1);
  while (ros::ok()){
  	  Voxel::logger.setDefaultLogLevel(LOG_INFO);
      //Create camera objects
      CameraSystem sys;
      DepthCameraPtr depthCamera;
      //Find TI devices connected to USB
      const Vector<DevicePtr> &devices = sys.scan();
      if (devices.size() > 0){
        // Connect to first available device
          depthCamera = sys.connect(devices[0]);
      }
      else{
          std::cerr << "SimplePCLViewer: Could not find a compatible device."
                                                                  << std::endl;
          return -1;
      }
      if (!depthCamera){
          std::cerr << "SimplePCLViewer: Could not open a depth camera."
                                                                  << std::endl;
          return -1;
      }
      std::cout << "Successfully loaded depth camera for device "
                                              << depthCamera->id() << std::endl;
      int frame_count = 100;
      Voxel::TimeStampType last_time_stamp = 0;
      std::string type = "pointcloud";
      //TOF type should be taw, if so step into callback function
      if (type == "pointcloud"){
        	 depthCamera->registerCallback(DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME, [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
           const XYZIPointCloudFrame *d = dynamic_cast<const XYZIPointCloudFrame *>(&frame);
           if(!d)
           {
             std::cout << "Null frame captured? or not of type XYZIPointCloudFrame" << std::endl;
             return -1;
           }
           std::cout << "Capture frame " << d->id << "@" << d->timestamp;
           //Print FPS
           if(last_time_stamp != 0){
             std::cout << " (" << 1E6/(d->timestamp - last_time_stamp)
                                                            << " fps)";
           }
           std::cout << std::endl;
           //Create new pointcloud
           PointCloud::Ptr msg (new PointCloud);
           msg->header.frame_id = "map";
           //Loop through all data
           float data_size = d->size();
           msg->points.resize(data_size);
           float int_max = d->points.at(0).i;
           for (int i=0;i<data_size; i++){
             float intensity = d->points.at(i).i;
             if (intensity > int_max){
               int_max = intensity;
             }
           }
           float z_max = d->points.at(0).z;
           for (int i = 0; i< data_size; i++){
             //Add points to pointcloud topic
             float intensity = d->points.at(i).i;
             float z = d->points.at(i).z;
             if (z > 0.3){
               float x = d->points.at(i).x;
               float y = d->points.at(i).y;
               if (z > z_max){
                 z_max = z;
               }
               msg->points[i].x = x;
               msg->points[i].y = y;
               msg->points[i].z = z;
               msg->points[i].intensity = intensity;
             }
            }
            tofPub.publish (msg);
            last_time_stamp = d->timestamp;
            ros::spinOnce();
        	});
        }
      else if (type == "depth"){
          depthCamera->registerCallback(DepthCamera::FRAME_DEPTH_FRAME,
            [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
            const DepthFrame *d = dynamic_cast<const DepthFrame *>(&frame);

            if (!d)
            {
              std::cout << "Null frame captured? or not of type DepthFrame"
                                                                << std::endl;
              return;
            }

            std::cout << "Capture frame" << d->id << "@" << d->timestamp;

            if (last_time_stamp != 0)
              std::cout << " (" << 1E6 / (d->timestamp - last_time_stamp)
                                                                << " fps)";

            std::cout << std::endl;

            last_time_stamp = d->timestamp;
          });
        }


        if(depthCamera->start()){
          FrameRate r;
          if(depthCamera->getFrameRate(r))
            logger(LOG_INFO) << "Capturing at a frame rate of "
                                    << r.getFrameRate() << " fps" << std::endl;
          depthCamera->wait();
        }
        else{
          logger(LOG_ERROR) << "Could not start the depth camera "
                                            << depthCamera->id() << std::endl;
        }
      }
      return 0;
}
