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
#include "std_msgs/Header.h"
#include <image_transport/image_transport.h>

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
using namespace cv;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef pcl::PointXYZI PointT;

Mat clipBackground(Mat _iMat, Mat _dMat, float dThr, float iThr){
  Mat dMat = Mat::zeros( _iMat.size(), CV_32FC1 );
  for (int i = 0; i < _dMat.rows; i++) {
    for (int j = 0; j < _dMat.cols; j++) {
      dMat.at<float>(i,j) = (_iMat.at<float>(i,j) > iThr) ? 255.0 : 0.0;
    }
  }
  return dMat;
}

int main(int argc, char** argv){
  Mat _dMat, _iMat, _bMat, _bkgndMat;
  //init ROS
  ros::init(argc, argv, "TOF_pointcloud_publisher", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher depth_pub = it.advertise("tof/depth/image", 1);
  int type = CV_32FC1;
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
           if(last_time_stamp != 0)
             std::cout << " (" << 1E6/(d->timestamp - last_time_stamp) << " fps)";
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
           //std::vector<float> zMap;
           //std::vector<float> iMap;
           //std::vector<PointT> depthMap; //USe this vector to remap to image
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
               //zMap.push_back(z);
               //iMap.push_back(intensity);
               if (z == 0){
                 std::cout << "ZEROOOOOOO" << std::endl;
               }
               msg->points[i].x = x;
               msg->points[i].y = y;
               msg->points[i].z = z;
               //std::cout << intensity/int_max << std::endl;
               //msg->points[i].g = (intensity/int_max)*255*2;
               //msg->points[i].b = (intensity/int_max)*255*3;
               //std::cout << msg->points[i] << std::endl;
               msg->points[i].intensity = intensity;
             }
            }
            //std::transform(zMap.begin(), zMap.end(), zMap.begin(),
               //std::bind(std::multiplies<float>(), std::placeholders::_1, 1.0/z_max));
            //std::transform(zMap.begin(), zMap.end(), zMap.begin(),
                //std::bind(std::multiplies<float>(), std::placeholders::_1, (255.0)));
            //Publish data
            //std::cout << msg->height << " " << msg->width << std::endl;
            pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
            tofPub.publish (msg);
            last_time_stamp = d->timestamp;
            ros::spinOnce();
            /*
            //std::cout << zMap.size() << std::endl;
            _dMat = Mat(300, 200, CV_32FC1, zMap.data());

            _iMat = Mat(300, 200, CV_32FC1, iMap.data());
            _dMat = Mat(300, 300, CV_32FC1, zMap.data());
            //Mat dMat = clipBackground(_iMat, _dMat, 2.0/100, 3.0/100);
            imshow( "Display window", _dMat );
            waitKey(1);
            //std::cout << _dMat;
            std_msgs::Header header;
            header.frame_id = "map";
            sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(header, "mono8", _dMat).toImageMsg();

            depth_pub.publish(depth_msg);
            */
        	});
        }
      else if (type == "depth"){
          depthCamera->registerCallback(DepthCamera::FRAME_DEPTH_FRAME, [&](DepthCamera &dc, const Frame &frame, DepthCamera::FrameType c) {
            const DepthFrame *d = dynamic_cast<const DepthFrame *>(&frame);

            if (!d)
            {
              std::cout << "Null frame captured? or not of type DepthFrame" << std::endl;
              return;
            }

            std::cout << "Capture frame" << d->id << "@" << d->timestamp;

            if (last_time_stamp != 0)
              std::cout << " (" << 1E6 / (d->timestamp - last_time_stamp) << " fps)";

            std::cout << std::endl;

            //f.write((char *)&d->id, sizeof(d->id));
            //f.write((char *)&d->timestamp, sizeof(d->timestamp));

            last_time_stamp = d->timestamp;
            /*
            PointCloud::Ptr msg (new PointCloud);
            msg->header.frame_id = "map";
            //Loop through all data
            float data_size = d->size.width * d->size.height;

            msg->points.resize(data_size);

            float int_max = d->points.at(0).i;
            for (int i=0;i<data_size; i++){
              float intensity = d->points.at(i).i;
              if (intensity > int_max){
                int_max = intensity;
              }
            }

            std::vector<float> depth = d->depth;
            */
            //std::cout<< d->depth << std::endl;
            //float z_max = d->points.at(0).z;
            /*
            for (int i = 0; i< data_size; i++){
              //Add points to pointcloud topic
              float intensity = d->points.at(i).i;
              float z = d->points.data().at(i).z;
              float x = d->points.data().at(i).x;
              float y = d->points.data().at(i).y;
              if (z > z_max){
                z_max = z;
              }
              msg->points[i].x = x;
              msg->points[i].y = y;
              msg->points[i].z = z;
              msg->points[i].intensity = intensity;
             }

             std::cout << msg->height << " " << msg->width << std::endl;
             pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
             tofPub.publish (msg);
             last_time_stamp = d->timestamp;
             ros::spinOnce();
             */
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
