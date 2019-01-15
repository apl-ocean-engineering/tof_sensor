#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl_ros/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include "Eigen/Core"
#include "Eigen/Geometry"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace cv;

class PointCloudToDepthImage{

  image_geometry::PinholeCameraModel cam_model;
  sensor_msgs::CameraInfo cam_info;
  PointCloudT cloud;
  Eigen::Matrix3d K;

  ros::NodeHandle n;
  ros::Subscriber pointcloud_sub = n.subscribe("tof/points/raw", 1000, &PointCloudToDepthImage::pointcloud_callback, this);
  public:
    PointCloudToDepthImage();
    void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void run();
};
PointCloudToDepthImage::PointCloudToDepthImage(){
  K = Eigen::Matrix3d::Zero(3,3);
  K(0,1) = 300.0;
  K(0,2) = 300.0;
  K(1,1) = 300.0;
  K(1,2) = 300.0;
  K(2,2) = 1.0;
  //cam_model.fromCameraInfo(cam_info);
}
void PointCloudToDepthImage::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg){
  //PointCloudT cloud;
  pcl::fromROSMsg(*msg, cloud);
}

void PointCloudToDepthImage::run(){
  ros::Rate loop_rate(50);
  PointCloudT::Ptr cloud2(new PointCloudT);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud2, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  //viewer->spinOnce (100);
  while (ros::ok()){
    if( !cloud.empty() ){
      //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
      //viewer = pcl::simpleVis(cloud);
      //PointCloudT::Ptr cloud2 = *cloud;
      *cloud2 = cloud;
      if (!viewer->wasStopped()){
        viewer->updatePointCloud<PointT> (cloud2, "sample cloud");
      }
      viewer->spinOnce();


      //boost::this_thread::sleep (boost::posix_time::microseconds (100000));


      /*
      Eigen::Vector3d resized(0.0, 320.0, 240.0); //(x_min, x_max, y_min, y_max) image size
      Eigen::Matrix3d M = Eigen::Matrix3d::Zero(); //Matrix consisting of min and max pointcloud distribution properties
      PointT min, max;
      pcl::getMinMax3D(cloud, min, max);

      M(0,0) = min.x;
      M(0,1) = min.z;
      M(1,0) = max.x;
      M(1,1) = max.z;
      M(2,0) = max.y;
      M(2,2) = max.z;
      Eigen::Vector3d intrinsics; //4 vector of intrinsics as (f_x, x0, f_y, y0)
      std::cout << "z: " << min.z << std::endl;
      intrinsics = M.inverse()*resized;

      std::cout << intrinsics << std::endl << std::endl;
      /*
      //Solve for a truncated K matrix to map image to 320X240
      //K_trunc(2X3) = Top two rows of K(3X3)
      Eigen::Matrix2d image_mins;
      image_mins(0,0) = 0;
      image_mins(1,0) = 320;
      image_mins(0,1) = 0;
      image_mins(1,1) = 240;
      //image_mins(0,2) = 0;
      //image_mins(1,2) = 240;
      PointT min, max;
      pcl::getMinMax3D(cloud, min, max);
      Eigen::Matrix2d image_maxs;
      image_maxs(0,0) = min.x;
      image_maxs(1,0) = max.x;
      image_maxs(0,1) = min.y;
      image_maxs(1,1) = max.y;
      Eigen::Matrix2d K_trunc =image_mins*image_maxs.completeOrthogonalDecomposition().pseudoInverse();;
      std::cout << "Mins: " << std::endl << image_mins << std::endl;
      std::cout << "Maxs: " << std::endl << image_maxs << std::endl;
      std::cout << "K_trunc: " << std::endl << K_trunc << std::endl;
      //std::cout << image_mins << " " << image_maxs << " " << K_trunc << std::endl;
      std::cout << std::endl;
      */

      /*
      PointCloudT projected_pointcloud;
      std::vector< PointT, Eigen::aligned_allocator<pcl::PointXYZ>> projected_points;
      for (int i =0;i<cloud.size();i++){
        PointT projected_point;
        Eigen::Vector3d point = Eigen::Vector3d(cloud.points.at(i).x, cloud.points.at(i).y, cloud.points.at(i).z);
        Eigen::Vector3d uv = K*point;
        projected_point.x =  uv[0];
        projected_point.y =  uv[1];
        projected_point.z =  uv[2];
        projected_points.push_back(projected_point);
      }
      projected_pointcloud.points = projected_points;
      PointT min, max;
      pcl::getMinMax3D(projected_pointcloud, min, max);
      //std::cout << min << " " << max << std::endl;

      int mat_max_X = (int) max.x/max.z;
      int mat_max_Y = (int) max.y/max.z;
      Mat image = cv::Mat::zeros(cv::Size(mat_max_X, mat_max_Y), CV_8U);
      //for (int i=0;i<projected_pointcloud.points.size();i++){
      for (int i=0;i<100000;i++){
        PointT point = projected_pointcloud.points.at(i);
        int x = (int) point.x/max.z;
        int y = (int) point.y/max.z;
        float _z = point.z;
        int z = (int) (z/max.z)*255;
        image.at<uchar>(x,y) = z;
      }
      //std::cout << image << std::endl;
      imshow("Img", image);
      waitKey(1);
      */
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
