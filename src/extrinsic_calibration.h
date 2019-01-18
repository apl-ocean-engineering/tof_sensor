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
  Eigen::MatrixXd Xd;//Checkerboard corners relative to depth camera frame
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
  Eigen::Matrix3d K;
  //ROS stuff
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber info_sub = nh_.subscribe("/camera/rgb/camera_info", 1000,
                                &ImageConverter::img_info_callback, this);
  //ros::Subscriber pointcloud_sub = nh_.subscribe("tof/points/raw", 1000,
  //                              &ImageConverter::pointcloud_callback, this);
  ros::Subscriber pointcloud_sub = nh_.subscribe("tof_pointcloud", 1000,
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
  std::tuple<Eigen::VectorXd, Eigen::MatrixXd> calculate_extrinsic_paramaters();
  void static_frame_transformation();
  void run();
};

ImageConverter::ImageConverter(): it_(nh_) {
  // Subscrive to input video feed and publish output video feed
  K = Eigen::Matrix3d::Zero(3, 3);
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

std::tuple<bool, Eigen::MatrixXd, Eigen::MatrixXd>
                ImageConverter::chess_board_points(cv_bridge::CvImagePtr cv){
  //Determine image plane coordinates of checkerboard corners
  std::vector<cv::Point2f> checkerboard_points_vector; //Vector to record points
  cv::Mat imgGray;
  Eigen::MatrixXd Xi; //Image plane homogenous points
  Eigen::MatrixXd Xw; //World plane homogenous points
  //Determine if camera can see points
  bool found = cv::findChessboardCorners(img, patternsize, checkerboard_points_vector,
                                                    chessBoardFlags);
  if (found){
    //Find and draw checkerboard points
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(imgGray, checkerboard_points_vector, cv::Size(5,5), cv::Size(-1,-1),
                cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT,
                30, 0.0001));
    drawChessboardCorners(img, patternsize, cv::Mat(checkerboard_points_vector), found);
    cv::waitKey(1);
    for (int i=0;i<checkerboard_points_vector.size(); i++){
      //Loop through vector and add points to inertial frame matrix
      cv::Point2f point = checkerboard_points_vector.at(i);
      Eigen::Vector3d X(point.x, point.y, 1);
      Xi.conservativeResize(3, Xi.cols()+1);
      Xi.col(Xi.cols()-1) = X;
    }
    float Z = 1.0;
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
  return std::make_tuple(found, Xi, Xw);
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

void ImageConverter::static_frame_transformation(){
  //Determine a scaled rotation and translation between frames from one static frame
  std::tuple<bool, Eigen::MatrixXd,Eigen::MatrixXd> chess_points = chess_board_points(cv_ptr);
  bool found = std::get<0>(chess_points);
  Eigen::MatrixXd Xi = std::get<1>(chess_points); //Image plane homogenous points
  Eigen::MatrixXd Xw = std::get<2>(chess_points); //World plane homogenous points

  /* 90% sure I don't need this here...
  if (Xd.cols() != Xv.cols()){
    //Need to select four points
    ROS_ERROR("UNEQUAL POINTS SELECTED IN DEPTH CAMERA.");
    return ;
  }
  */

  if (!found){
    ROS_ERROR("CHECKERBOARD NOT VISIBLE");
    return ;
  }

  if (Xw.rows() != 4){
    ROS_ERROR("MUST SELCT THE FOUR CHECKERBOARD CORNERS");
    return ;
  }
  //Get camera transformation
  Eigen::MatrixXd Xi_pinv = Xi.completeOrthogonalDecomposition().pseudoInverse();
  //Find the inverse transformation, transform back to frame
  Eigen::MatrixXd color_camera_transformation =
              (Xw*Xi_pinv*K).completeOrthogonalDecomposition().pseudoInverse();
  //Calculate (Col 3 is all zeros due to depth ambiguity)
  Eigen::MatrixXd Xv_pinv = Xv.completeOrthogonalDecomposition().pseudoInverse();
  Eigen::MatrixXd depth_camera_transformation = Xd*Xv_pinv;
  //Record transformation
  camera_transformation_vector.push_back(color_camera_transformation);
  depth_transformation_vector.push_back(depth_camera_transformation);
  ROS_INFO("POINTS RECORDED");
}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd>
                              ImageConverter::calculate_extrinsic_paramaters(){
  //Determine extrinsic calibration from series of frame transformation
  Eigen::VectorXd translation;
  Eigen::MatrixXd rotation;
  if (camera_transformation_vector.size() != depth_transformation_vector.size()){
    //Verify equal transformations were preformed
    ROS_ERROR("UNEQUAL EXTRINSIC CALIBRATION EVENTS");
    return std::make_tuple(translation, rotation);
  }
  Eigen::MatrixXd Mc; //3XN
  Eigen::MatrixXd Md;
  Eigen::MatrixXd b_c; //1XN
  Eigen::MatrixXd b_d;
  //Populate 'M', and 'b' matricies to calculate transformation
  for (int i=0; i<camera_transformation_vector.size(); i++){
    //Get transformation matricies
    Eigen::MatrixXd Gc = camera_transformation_vector.at(i);
    Eigen::MatrixXd Gd = depth_transformation_vector.at(i);
    //Isolate 't' vectors
    Eigen::Vector3d t_c(Gc(3,0), Gc(3,1), Gc(3,2));
    Eigen::Vector3d t_d(Gd(3,0), Gd(3,1), Gd(3,2));
    //M matricies
    Eigen::Vector3d r_c(Gc(2,0), Gc(2,1), Gc(2,2));
    Mc.conservativeResize(3, Mc.cols()+1);
    Mc.col(i) = r_c;
    Eigen::Vector3d r_d(Gd(2,0), Gd(2,1), Gd(2,2));
    Md.conservativeResize(3, Md.cols()+1);
    Md.col(i) = r_d;
    //b vectors
    b_c.conservativeResize(1, b_c.cols()+1);
    b_c.col(i) = r_c.transpose()*t_c;
    b_d.conservativeResize(1, b_d.cols()+1);
    b_d.col(i) = r_d.transpose()*t_d;
  }
  //Calculate rotation and transformation
  translation = (Mc*Mc.transpose()).inverse()*Mc*(b_c-b_d); //We got translation!
  Eigen::MatrixXd R = Md*(Mc.transpose()); //Don't quite have R though...
  //Need to adjust for error. Turn R into orthonormal matrix through svd...
  //...with S matrix == I
  Eigen::JacobiSVD<Eigen::MatrixXd>
                              svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
  rotation = svd.matrixU()*(svd.matrixV().transpose()); //Final rotation
  return std::make_tuple(translation, rotation);


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

      if (key_press == "r"){
        //If r has been pushed, remove most recent entry
        Xd.conservativeResize(Xd.rows(), Xd.cols()-1);
        key_press = " ";
      }

      else if (key_press == "s"){
        //If s has been pushed, calculate [R,T] for 1 frame, for for each
        //of the sensors to their respective frames
        static_frame_transformation();
        Xd.conservativeResize(0,0);
        key_press = " ";
      }

      else if (key_press == "f"){
        //If f has been pushed, calculate full transformation
        std::tuple<Eigen::VectorXd,Eigen::MatrixXd> G =
                                              calculate_extrinsic_paramaters();
        Eigen::VectorXd t = std::get<0>(G); //Translation
        Eigen::MatrixXd R = std::get<1>(G); //Rotation
        std::cout << "R: " << R << std::endl;
        std::cout << "t: " << t << std::endl;
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
