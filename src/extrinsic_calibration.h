#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float64.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <Eigen/Dense>
#include <Eigen/Eigen>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string TRACKBAR_NAME = "Trackbar";

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace cv;

class ExtrinsicCalibration {
  cv::Mat img;
  const cv::Size patternsize = cv::Size(8, 6);
  const float checkerboardDist = .035; // m
  double check_x = 0.455;              // m
  double check_y = 0.3;                // m
  int chessBoardFlags;

  Eigen::MatrixXd Xv; // Checkerboard corners relative to checkerboard frame
  Eigen::MatrixXd Xd; // Checkerboard corners relative to depth camera frame
  cv_bridge::CvImagePtr cv_ptr; // Cv image pointer, for ROS callback
  std::string key_press; // Initalize variable to record which key was pressed
  std::vector<Eigen::MatrixXd> camera_transformation_vector;
  std::vector<Eigen::MatrixXd> depth_transformation_vector;

  // Trackbar (all in cm)
  int x_min = 0;
  int x_max = 2000;
  int y_min = 0;
  int y_max = 2000;
  int z_min = 0;
  int z_max = 2000;
  char TrackbarName[50];

  // Initalize pointcloud variables
  PointCloudT cloud;
  PointT clickedPoint;

  // Camera intrinsics
  std_msgs::Float64 fx;
  std_msgs::Float64 fy;
  std_msgs::Float64 s;
  std_msgs::Float64 cx;
  std_msgs::Float64 cy;
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero(3, 3);
  // ROS stuff
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ExtrinsicCalibration(std::string image_topic);
  ~ExtrinsicCalibration();

  // ROS Callbacks
  void imageCb(const sensor_msgs::ImageConstPtr &msg);
  void img_info_callback(const sensor_msgs::CameraInfo::ConstPtr &info);
  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);

  // PCL callbacks
  void points3D_callback(const pcl::visualization::PointPickingEvent &event,
                         void *);
  void keyboard_callback(const pcl::visualization::KeyboardEvent &event,
                         void *);

  // Tranformation helper functions
  std::vector<Point2f> matrix_to_vector(Eigen::MatrixXd X);
  std::tuple<bool, Eigen::MatrixXd, Eigen::MatrixXd> chess_board_points();
  std::tuple<Mat, Mat> transformation_from_homography(
      const std::vector<Mat> rotations, const std::vector<Mat> translations,
      const std::vector<Mat> normals, const int board);
  std::tuple<bool, Eigen::Matrix4d>
  planar_transformation(const std::vector<Point2f> src_vec,
                        const std::vector<Point2f> dist_vec, const int board);

  // Main functions
  std::tuple<Eigen::VectorXd, Eigen::MatrixXd> calculate_extrinsic_paramaters();
  void static_frame_transformation();
  void run();
};
