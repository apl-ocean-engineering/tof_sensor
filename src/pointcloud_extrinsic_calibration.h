#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/random_sample.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <Eigen/Dense>
#include <Eigen/Eigen>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class PointcloudExtrinsic {

  PointCloudT cloudLeft;
  PointCloudT cloudRight;

  Eigen::MatrixXf leftPc_points;
  Eigen::MatrixXf rightPc_points;
  bool update_left;
  bool update_right;

  std::string key_press; // Initalize variable to record which key was pressed

public:
  PointcloudExtrinsic();
  ~PointcloudExtrinsic();

  void PointcloudLeftCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void PointcloudRigtCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void points3D_callbackLeft(const pcl::visualization::PointPickingEvent &event,
                             void *);
  void
  points3D_callbackRight(const pcl::visualization::PointPickingEvent &event,
                         void *);
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  createViewer(std::string name, PointCloudT::Ptr inputCloud);
  void keyboard_callback(const pcl::visualization::KeyboardEvent &event,
                         void *);
  void run();
};
