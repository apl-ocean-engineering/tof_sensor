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

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/ChannelFloat32.h"

using namespace Voxel;

int main(int argc, char** argv){
  //init ROS
  ros::init(argc, argv, "TOF_Node", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("TOF_pointcloud", 1000);
  ros::Rate loop_rate(10);
  //Verify ROS is working properly
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
           //Enter pointcloud callback
        	 depthCamera->registerCallback(DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME, [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
           const XYZIPointCloudFrame *d = dynamic_cast<const XYZIPointCloudFrame *>(&frame);
           if(!d)
           {
             std::cout << "Null frame captured? or not of type XYZIPointCloudFrame" << std::endl;
             return -1;
           }
           //ROS pointcloud message
           sensor_msgs::PointCloud pointcloud;
           std_msgs::Header header;
           header.frame_id = "map";
           pointcloud.header = header;
           std::vector<geometry_msgs::Point32> points;
           std::vector<float> intensityData;
           //Loop through points and add to pointcloud
           int num = 0;
        	 for (int i = 0; i< d->size(); i++){
               geometry_msgs::Point32 point;
        			 float x = d->points.at(i).x;
        			 float y = d->points.at(i).y;
        			 float z = d->points.at(i).z;
               float intensity = d->points.at(i).i;
               if (intensity > 0.0 && num % 5 == 0){
                 point.x = x;
                 point.y = y;
                 point.z = z;
                 points.push_back(point);
                 intensityData.push_back(intensity);
               }
               num ++;

        	 }
           //Print frame info to console
           std::cout << "Capture frame " << d->id << "@" << d->timestamp;
           //Print FPS
           if(lastTimeStamp != 0)
             std::cout << " (" << 1E6/(d->timestamp - lastTimeStamp) << " fps)";
           std::cout << std::endl;
           lastTimeStamp = d->timestamp;
           // Publish ROS info
           pointcloud.points = points;
           sensor_msgs::ChannelFloat32 intensityChannelMsg;
           intensityChannelMsg.name = "intensity";
           intensityChannelMsg.values = intensityData;
           std::vector<sensor_msgs::ChannelFloat32> intensityChannelVector;
           intensityChannelVector.push_back(intensityChannelMsg);
           pointcloud.channels = intensityChannelVector;
           pointcloud_pub.publish(pointcloud);
           count++;
           ros::spinOnce();

           // TODO: Add expcetion handeler

           //if(count >= frameCount)
          //   dc.stop();
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
