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
static const std::string TRACKBAR_NAME = "Trackbar";

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace cv;

class ImageConverter
{
  cv::Mat img;
  cv::Size patternsize = cv::Size(8,6);
  float checkerboardDist = .04826; //m
  int chessBoardFlags;

  Eigen::MatrixXd Xv; //Checkerboard corners relative to checkerboard frame
  Eigen::MatrixXd Xd; //Checkerboard corners relative to depth camera frame
  Eigen::MatrixXd Xi; //Checkerboard corners relative to image frame
  Eigen::MatrixXd Xw; //Checkerboard corners relative to depth camera world frame
  cv_bridge::CvImagePtr cv_ptr; //Cv image pointer, for ROS callback
  std::string key_press; //Initalize variable to record which key was pressed
  std::vector<Eigen::MatrixXd> camera_transformation_vector;
  std::vector<Eigen::MatrixXd> depth_transformation_vector;

  //Trackbar (all in cm)
  int x_min = 0;
  int x_max = 2000;
  int y_min = 0;
  int y_max = 2000;
  int z_min = 0;
  int z_max = 2000;
  char TrackbarName[50];
  //Initalize pointcloud variables
  PointCloudT cloud;
  PointT clickedPoint;
  //Camera intrinsics
  std_msgs::Float64 fx;
  std_msgs::Float64 fy;
  std_msgs::Float64 s;
  std_msgs::Float64 cx;
  std_msgs::Float64 cy;
  Eigen::MatrixXd K;
  //ROS stuff
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber info_sub = nh_.subscribe("/camera/rgb/camera_info", 1000,
                                &ImageConverter::img_info_callback, this);
  ros::Subscriber pointcloud_sub = nh_.subscribe("/camera/depth/points", 1000,
                                &ImageConverter::pointcloud_callback, this);

public:
  ImageConverter();
  ~ImageConverter();
  std::tuple<bool, Eigen::MatrixXd, Eigen::MatrixXd>
                                  chess_board_points(cv_bridge::CvImagePtr cv);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void img_info_callback(const sensor_msgs::CameraInfo::ConstPtr& info);
  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void mouse_callback(const pcl::visualization::MouseEvent &event,
                                                          void* viewer_void);
  void points3D_callback(const pcl::visualization::PointPickingEvent& event, void*);
  void keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*);
  //std::tuple<Eigen::VectorXd, Eigen::MatrixXd> calculate_extrinsic_paramaters();
  //void static_frame_transformation();
  static void mouse_wrapper(int event, int x, int y, int flags, void *param);
  void image_callback(int event, int x, int y, int flags, void *param);
  void run();
};

ImageConverter::ImageConverter(): it_(nh_) {
  // Subscrive to input video feed and publish output video feed
  K = Eigen::MatrixXd::Zero(3, 3);
  //Create board point vector, counterclockwise from bottom left
  //Just estimating for now...
  Xv.conservativeResize(4, Xv.cols()+1);
  Xv.col(Xv.cols()-1) =  Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
  Xv.conservativeResize(4, Xv.cols()+1);
  Xv.col(Xv.cols()-1) = Eigen::Vector4d(0.0, 36.0, 0.0, 1.0);
  Xv.conservativeResize(4, Xv.cols()+1);
  Xv.col(Xv.cols()-1) = Eigen::Vector4d(41.0, 36.0, 0.0, 1.0);
  Xv.conservativeResize(4, Xv.cols()+1);
  Xv.col(Xv.cols()-1) = Eigen::Vector4d(41.0, 0.0, 0.0, 1.0);

  chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
  image_sub_ = it_.subscribe("/camera/rgb/image_color", 1,
                          &ImageConverter::imageCb, this);
  cv::namedWindow(OPENCV_WINDOW);
}

ImageConverter::~ImageConverter(){
  cv::destroyWindow(OPENCV_WINDOW);
}

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    img = cv_ptr->image;
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void ImageConverter::pointcloud_callback(const
                                  sensor_msgs::PointCloud2ConstPtr& msg){
  //Turn ROS pointcloud into pcl pointcloud
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

void ImageConverter::mouse_callback(const pcl::visualization::MouseEvent
                                                    &event, void*){
  //2D mouse click callback
  if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
      event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease){
        std::cout << "Left mouse button released at position ("
                      << event.getX() << ", " << event.getY() << ")"
                      << std::endl;
        clickedPoint.x = event.getX();
        clickedPoint.y = event.getY();
      }
}

void ImageConverter::points3D_callback
                    (const pcl::visualization::PointPickingEvent& event, void*){
  //Get 3D point click information
  float x, y, z;
  if (event.getPointIndex () != -1){
    //If clicked, get point
    event.getPoint(x, y, z);
    ROS_INFO("Point selected: ");
    std::cout << "(x: " << x << " y: " << y << " z: " << z << ")" << std::endl;
    ROS_INFO("Press r to remove the last point");
    //Add point to Depth sensor matrix
    Xd.conservativeResize(4, Xd.cols()+1);
    Xd.col(Xd.cols()-1) =  Eigen::Vector4d(x, y, z, 1.0);
  }
}

void ImageConverter::keyboard_callback
                      (const pcl::visualization::KeyboardEvent& event, void* ){
  //Key press callback
  if( event.getKeyCode() && event.keyDown()){
    //If pressend, get which key was pressed
    key_press = event.getKeyCode();
  }
}

void ImageConverter::image_callback(int event, int x, int y, int, void* ){
  //ImageConverter* settings = reinterpret_cast<ImageConverter*>(userdata);
  if(event == EVENT_LBUTTONDOWN ){
    std::cout << "(x,y): "<< "(" << x << "," << y << ")" << std::endl;
    Xi.conservativeResize(3, Xi.cols()+1);
    Xi.col(Xi.cols()-1) =  Eigen::Vector3d(x, y, 1.0);
  }
}

void ImageConverter::mouse_wrapper(int event, int x, int y, int flags, void *param)
{
    ImageConverter * imgC = (ImageConverter*)param; // cast back to 'this'
    imgC->image_callback(event, x, y,flags, 0) ;
}

void ImageConverter::run(){
  ros::Rate loop_rate(50);
  //initalize cloud and viewer
  PointCloudT::Ptr cloud_ptr(new PointCloudT);
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
                  viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //Set basic cloud properties
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud_ptr, "sample cloud");
  viewer->setPointCloudRenderingProperties
            (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  //Register the callbacks we need
  viewer->registerPointPickingCallback(&ImageConverter::points3D_callback, *this);
  viewer->registerKeyboardCallback(&ImageConverter::keyboard_callback, *this);

  setMouseCallback(OPENCV_WINDOW, mouse_wrapper, this);
  //trackbar init
  //OpenCV doesn't allow for negative values :(
  namedWindow(TRACKBAR_NAME, 1);
  createTrackbar("x_min ([-1000, 1000] cm)", TRACKBAR_NAME, &x_min, 2000);
  createTrackbar("x_max ([-1000, 1000] cm)", TRACKBAR_NAME, &x_max, 2000);
  createTrackbar("y_min ([-1000, 1000] cm)", TRACKBAR_NAME, &y_min, 2000);
  createTrackbar("y_max ([-1000, 1000] cm)", TRACKBAR_NAME, &y_max, 2000);
  createTrackbar("z_min ([-1000, 1000] cm)", TRACKBAR_NAME, &z_min, 2000);
  createTrackbar("z_max ([-1000, 1000] cm)", TRACKBAR_NAME, &z_max, 2000);

  ROS_INFO("CLICK POINTS ON BOARD. PRESS 's' TO RECORD POINTS FOR 1 STATIC FRAME");
  ROS_INFO("Click clockwise from bottom left for now...");
  ROS_INFO("PRESS F TO FINISH AND CALCULATE TRANSFORMATION");
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
      pcl::PassThrough<pcl::PointXYZ> pass;
      //x filter
      pass.setInputCloud(cloud_ptr);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits(x_min_float, x_max_float);
      pass.filter(*cloud_filtered);
      //y filter
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits(y_min_float, y_max_float);
      pass.filter(*cloud_filtered);
      //z filter
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits(z_min_float, z_max_float);
      pass.filter(*cloud_filtered);
      //Update cloud with most recent from callback
      viewer->updatePointCloud<PointT> (cloud_filtered, "sample cloud");
      /*
      if (key_press == "r"){
        //If r has been pushed, remove most recent entry
        Xd.conservativeResize(Xd.rows(), Xd.cols()-1);
        key_press = " ";
      }

      else if (key_press == "s"){
        //If s has been pushed, calculate [R,T] for 1 frame, for for each
        //of the sensors to their respective frames
        //static_frame_transformation();
        //Xd.conservativeResize(0,0);
        key_press = " ";
      }
      */
      if (key_press == "f"){
        std::cout << "Xi: " << std::endl << Xi << std::endl;
        std::cout << "Xd: " << std::endl << Xd<< std::endl;
        Eigen::MatrixXd Xd_pinv = Xd.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd K_pinv = K.inverse();//K.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd G_ = K_pinv*Xi*Xd_pinv;
        //std::cout << "Xi*Xd^(-1): " << std::endl << Xd.completeOrthogonalDecomposition().pseudoInverse() << std::endl;
        std::cout << "Xd_pinv: " << std::endl << Xd_pinv <<std::endl;
        std::cout << G_.rows() << "," << G_.cols() << std::endl;
        std::cout << "G_: " << std::endl << G_ <<std::endl;
        //std::cout << "P_K: " << std::endl << K_pinv <<std::endl;
        //std::cout << "GP_K: " << std::endl << K*G_ <<std::endl;
        key_press = " ";
      }

      if (img.rows > 60 && img.cols > 60){
        //Show the image
        cv::imshow(OPENCV_WINDOW, img);
      }
    waitKey(1);
    ros::spinOnce();
    viewer->spinOnce();

    loop_rate.sleep();

  }
  }
}
