#include <iostream>
#include <vector>
#include <stdio.h>
#include <termios.h>

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float64.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <pcl_ros/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <boost/foreach.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <Eigen/Dense>
#include <Eigen/Eigen>
#include "Eigen/Core"
#include "Eigen/Geometry"

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace cv;

class ImageConverter
{
  cv::Mat img;
  cv::Size patternsize = cv::Size(8,6);
  float checkerboardDist = .04826; //m
  cv_bridge::CvImagePtr cv_ptr;
  int point_press_count = 0;
  bool print_point = true;

  Eigen::MatrixXd Xi; //Image plane homogenous points
  Eigen::MatrixXd Xw; //Image plane homogenous points

  //Trackbar (all in cm)
  int x_min = 1000;
  int x_max = 1000;
  int y_min = 1000;
  int y_max = 1000;
  int z_min = 1000;
  int z_max = 1000;
  const std::string trackbar_name = "trackbar";

  char TrackbarName[50];

  PointCloudT cloud;
  PointT clickedPoint;

  std_msgs::Float64 fx;
  std_msgs::Float64 fy;
  std_msgs::Float64 s;
  std_msgs::Float64 cx;
  std_msgs::Float64 cy;
  Eigen::Matrix3d K;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int chessBoardFlags;

  ros::Subscriber info_sub = nh_.subscribe("/camera/rgb/camera_info", 1000,
                                &ImageConverter::img_info_callback, this);
  ros::Subscriber pointcloud_sub = nh_.subscribe("tof/points/raw", 1000,
                                &ImageConverter::pointcloud_callback, this);

public:
  ImageConverter();
  ~ImageConverter();
  void chessBoard(cv_bridge::CvImagePtr cv);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void img_info_callback(const sensor_msgs::CameraInfo::ConstPtr& info);
  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void mouseEventOccurred(const pcl::visualization::MouseEvent &event,
                                                          void* viewer_void);
  void points3D(const pcl::visualization::PointPickingEvent& event, void*);
  void record_point();
  int getch();
  void run();
};

ImageConverter::ImageConverter(): it_(nh_) {
  // Subscrive to input video feed and publish output video feed
  K = Eigen::Matrix3d::Zero(3, 3);
  chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
                          &ImageConverter::imageCb, this);
  cv::namedWindow(OPENCV_WINDOW);
}

ImageConverter::~ImageConverter(){
  cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::chessBoard(cv_bridge::CvImagePtr cv){
  std::vector<cv::Point2f> pointBuf;
  img = cv->image;
  cv::Mat imgGray;
  bool found = cv::findChessboardCorners(img, patternsize, pointBuf,
                                                    chessBoardFlags);
  if (found){
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(imgGray, pointBuf, cv::Size(5,5), cv::Size(-1,-1),
                cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT,
                30, 0.0001));
    drawChessboardCorners(img, patternsize, cv::Mat(pointBuf), found);
    int k = cv::waitKey(3);
    //Construct image points (Xi)
    if (10 == 10){
      for (int i=0;i<pointBuf.size(); i++){
        cv::Point2f point = pointBuf.at(i);
        Eigen::Vector3d X(point.x, point.y, 1);
        Xi.conservativeResize(4, Xi.cols()+1);
        Xi.col(Xi.cols()-1) = X;
      }
      float Z = 0.0;
      float w = 1.0;
      //Construct world points Xw
      for (int i=0; i<patternsize.height; i++){
        for(int j=0; j<patternsize.width; j++){
          float X = j*checkerboardDist;
          float Y = i*checkerboardDist;
          Eigen::Vector4d Xp(X,Y,Z,w);

          Xw.conservativeResize(4, Xw.cols()+1);
          Xw.col(Xw.cols()-1) = Xp;
        }
      }
    }
  }
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void ImageConverter::pointcloud_callback(const
                                  sensor_msgs::PointCloud2ConstPtr& msg){
  pcl::fromROSMsg(*msg, cloud);

}

void ImageConverter::img_info_callback(const
                                sensor_msgs::CameraInfo::ConstPtr& info){
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

void ImageConverter::mouseEventOccurred(const pcl::visualization::MouseEvent
                                                    &event, void*){
  if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
      event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease &&
      point_press_count == 0){
        std::cout << "Left mouse button released at position ("
                      << event.getX() << ", " << event.getY() << ")"
                      << std::endl;
        point_press_count += 1;
        clickedPoint.x = event.getX();
        clickedPoint.y = event.getY();
      }
}

//Make this function similar to mouseclick
void ImageConverter::points3D(const pcl::visualization::PointPickingEvent& event,
                                                                        void*){
  float x, y, z;
  if (event.getPointIndex () != -1 && point_press_count==0)
  {
    event.getPoint(x, y, z);
    std::cout << "Point clicked: ";
    std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
    clickedPoint.x = x;
    clickedPoint.y = y;
    clickedPoint.z = z;
    point_press_count += 1;
  }
}

void ImageConverter::record_point(){

}

int ImageConverter::getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 0;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

void ImageConverter::run(){
  ros::Rate loop_rate(50);
  //initalize cloud and viewer
  PointCloudT::Ptr cloud_ptr(new PointCloudT);
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
                  viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_ptr, "sample cloud");
  viewer->setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  //trackbar init
  namedWindow(trackbar_name, 1);
  createTrackbar("x_min ([-1000, 1000] cm)", trackbar_name, &x_min, 2000);
  createTrackbar("x_max ([-1000, 1000] cm)", trackbar_name, &x_max, 2000);
  createTrackbar("y_min ([-1000, 1000] cm)", trackbar_name, &y_min, 2000);
  createTrackbar("y_max ([-1000, 1000] cm)", trackbar_name, &y_max, 2000);
  createTrackbar("z_min ([-1000, 1000] cm)", trackbar_name, &z_min, 2000);
  createTrackbar("z_max ([-1000, 1000] cm)", trackbar_name, &z_max, 2000);
  while(ros::ok()){

    *cloud_ptr = cloud;
    //get mins and maxes from trackbar
    if (!viewer->wasStopped()){
      float x_min_float = (float) x_min;
      float x_max_float = (float) x_max;
      float y_min_float = (float) y_min;
      float y_max_float = (float) y_max;
      float z_min_float = (float) z_min;
      float z_max_float = (float) z_max;
      //resize to [-10,10]m
      x_min_float = (x_min_float - 1000.0)/100.0;
      x_max_float = (x_max_float - 1000.0)/100.0;
      y_min_float = (y_min_float - 1000.0)/100.0;
      y_max_float = (y_max_float - 1000.0)/100.0;
      z_min_float = (z_min_float - 1000.0)/100.0;
      z_max_float = (z_max_float - 1000.0)/100.0;
      //filter cloud
      //std::cout << "x min: " << x_min_float << "x_max: " << x_max_float << std::endl;
      pcl::PassThrough<pcl::PointXYZ> pass;
      //x filter
      pass.setInputCloud(cloud_ptr);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits(x_min_float, x_max_float);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      //y filter
      pass.setFilterFieldName ("y");
      pass.setFilterLimits(y_min_float, y_max_float);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      //z filter
      pass.setFilterFieldName ("z");
      pass.setFilterLimits(z_min_float, z_max_float);
      pass.filter(*cloud_filtered);

      viewer->updatePointCloud<PointT> (cloud_filtered, "sample cloud");

      viewer->registerPointPickingCallback(&ImageConverter::points3D, *this);
    }

    if (point_press_count == 1 && print_point){
      std::cout <<"PRESS ENTER TO SAVE THIS DATA" << std::endl;
      print_point = false;
    }
    if (point_press_count == 1){
      int input;
      input = getch();
      if (input == 10){
        //Enter pressed, record data
        record_point();
        print_point = true;
        point_press_count = 0;
      }
    }


    //int c = getch();
    //if (c == 10){
      //Enter has been pushed, save!
    //}

    ///////////////////SOLVE FOR EXTRINSICS//////////////////////////////
    /*
    if (img.rows > 60 && img.cols > 60){
      cv::imshow(OPENCV_WINDOW, img);
      int k = cv::waitKey(1);
      if (Xw.rows() == 4 && k ==10){
        Eigen::MatrixXd Xw_pinv = Xw.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd transformation = K.inverse()*Xi*Xw_pinv;
        //std::cout << transformation << std::endl;
      }
    }
    */
    waitKey(1);
    ros::spinOnce();
    viewer->spinOnce();

    loop_rate.sleep();

  }
}
