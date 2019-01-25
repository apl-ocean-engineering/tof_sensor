#include <iostream>
#include <vector>
#include <stdio.h>
#include <termios.h>
#include <stdlib.h>

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

#include <opencv2/core/eigen.hpp>
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
  cv::Size patternsize = cv::Size(6,8);
  float checkerboardDist = .04826; //m
  int chessBoardFlags;

  Eigen::MatrixXd Xv; //Checkerboard corners relative to checkerboard frame
  Eigen::MatrixXd Xd;//Checkerboard corners relative to depth camera frame
  cv_bridge::CvImagePtr cv_ptr; //Cv image pointer, for ROS callback
  std::string key_press; //Initalize variable to record which key was pressed
  std::vector<Eigen::MatrixXd> camera_transformation_vector;
  std::vector<Eigen::MatrixXd> depth_transformation_vector;
  Eigen::Matrix3d coordinate_frame_transformation;

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
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero(3, 3);
  //ROS stuff
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber info_sub = nh_.subscribe("/camera/rgb/camera_info", 1000,
                                &ImageConverter::img_info_callback, this);
  //ros::Subscriber pointcloud_sub = nh_.subscribe("tof/points/raw", 1000,
  //                              &ImageConverter::pointcloud_callback, this);
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
  std::tuple<Eigen::VectorXd, Eigen::MatrixXd> calculate_extrinsic_paramaters();
  void static_frame_transformation();
  std::tuple<Mat, Mat>  transformation_from_homography(std::vector<Mat> rotations, std::vector<Mat> translations, std::vector<Mat> normals, int board);
  std::tuple<bool,Eigen::Matrix4d> planar_transformation(std::vector<Point2f> src_vec, std::vector<Point2f> dist_vec, int board);
  std::vector<Point2f> matrix_to_vector(Eigen::MatrixXd X);
  void run();
};

ImageConverter::ImageConverter(): it_(nh_) {
  coordinate_frame_transformation << 1,0,0,0,1,0,0,0,1;
  // Subscrive to input video feed and publish output video feed
  //K = Eigen::Matrix3d::Zero(3, 3);
  //Create board point vector, counterclockwise from bottom left
  //Just estimating for now...
  double check_x = 0.6604; //41
  double check_y = 0.8382; //36
  Xv.conservativeResize(4, Xv.cols()+1);
  Xv.col(Xv.cols()-1) =  Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
  Xv.conservativeResize(4, Xv.cols()+1);
  Xv.col(Xv.cols()-1) = Eigen::Vector4d(0.0, check_y, 0.0, 1.0);
  Xv.conservativeResize(4, Xv.cols()+1);
  Xv.col(Xv.cols()-1) = Eigen::Vector4d(check_x, check_y, 0.0, 1.0);
  Xv.conservativeResize(4, Xv.cols()+1);
  Xv.col(Xv.cols()-1) = Eigen::Vector4d(check_x, 0.0, 0.0, 1.0);
  /*
  Xv.conservativeResize(3, Xv.cols()+1);
  Xv.col(Xv.cols()-1) =  Eigen::Vector3d(0.0, 0.0, 1.0);
  Xv.conservativeResize(3, Xv.cols()+1);
  Xv.col(Xv.cols()-1) = Eigen::Vector3d(0.0, check_y, 1.0);
  Xv.conservativeResize(3, Xv.cols()+1);
  Xv.col(Xv.cols()-1) = Eigen::Vector3d(check_x, check_y, 1.0);
  Xv.conservativeResize(3, Xv.cols()+1);
  Xv.col(Xv.cols()-1) = Eigen::Vector3d(check_x, 0.0, 1.0);
  */

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
    Xd.conservativeResize(3, Xd.cols()+1);
    Eigen::Vector3d x_(x/z, y/z, 1.0);
    Eigen::Vector3d x_hat = coordinate_frame_transformation*x_;
    Xd.col(Xd.cols()-1) =  x_hat;
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

std::vector<Point2f> ImageConverter::matrix_to_vector(Eigen::MatrixXd X){
  std::vector<Point2f> X_vec;

  for (int i=0; i<X.cols(); i++){
    Point2f p;
    p.x = X(0, i);
    p.y = X(1, i);
    X_vec.push_back(p);
  }

  return X_vec;
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
    //std::cout << checkerboard_points_vector << std::endl;
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(imgGray, checkerboard_points_vector, cv::Size(5,5), cv::Size(-1,-1),
                cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT,
                30, 0.0001));
    drawChessboardCorners(img, patternsize, cv::Mat(checkerboard_points_vector), found);
    std::vector<cv::Point2f> checkerboard_points_vector_meters;
    //TODO compress this loops
    for (int i=0;i<checkerboard_points_vector.size();i++){
      Point2f p;
      Eigen::Vector3d X_px(checkerboard_points_vector.at(i).x, checkerboard_points_vector.at(i).y, 1);
      Eigen::Vector3d X_m = K.inverse()*X_px;
      p.x = X_m(0);
      p.y = X_m(1);
      //p.x = X_px(0);
      //p.y = X_px(1);
      checkerboard_points_vector_meters.push_back(p);
      //std::cout << i << std::endl;
      //circle(img, checkerboard_points_vector.at(i), 10, Scalar(0, 255, 255));
      //imshow(OPENCV_WINDOW, img);
      //waitKey(100);
    }


    cv::waitKey(1);
    for (int i=0;i<checkerboard_points_vector_meters.size(); i++){
      //Loop through vector and add points to inertial frame matrix
      cv::Point2f point = checkerboard_points_vector_meters.at(i);
      Eigen::Vector3d X(point.x, point.y, 1);
      Xi.conservativeResize(3, Xi.cols()+1);
      Xi.col(Xi.cols()-1) = X;
    }
    float Z = 0.0;
    float w = 1.0;
    //Construct world points Xw
    for (int i=0; i<patternsize.height; i++){
      for(int j=0; j<patternsize.width; j++){
        float X = j*checkerboardDist;
        float Y = patternsize.height*checkerboardDist - (i+1)*checkerboardDist;
        Eigen::Vector3d Xp(X,Y,w);
        Xw.conservativeResize(3, Xw.cols()+1);
        Xw.col(Xw.cols()-1) = Xp;
      }
    }
  }
  //std::cout << Xw << std::endl;
  return std::make_tuple(found, Xi, Xw);
}

std::tuple<bool,Eigen::Matrix4d> ImageConverter::planar_transformation(std::vector<Point2f> src_vec, std::vector<Point2f> dst_vec, int board){

  Eigen::Matrix4d G = Eigen::Matrix4d::Zero(4,4);
  if (src_vec.size() != dst_vec.size()){
    ROS_ERROR("CANNOT CALCULATE HOMOGRAPHY WITH UNEQUAL POINTS");
    return std::make_tuple(false,G);
  }
  /************
  FOR DEBUGGING
  ************/
  std::vector<Point2f> pass_vec;
  for (int i=0; i<src_vec.size(); i++){
    Point2f p;
    float r1 = -0.001 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(.002)));
    float r2 = -0.001 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(.002)));
    //double r1d = (double) r1/10.0;
    //double r2d = (double) r2/10.0;
    p.x = src_vec.at(i).x + r1;
    p.y = src_vec.at(i).y + r2;
    pass_vec.push_back(p);
  }
  /**/
  Mat H = findHomography(src_vec, dst_vec);
  std::cout << "src_vec:  " << endl <<  src_vec << std::endl << std::endl;
  std::cout << "dst_vec:  " << endl <<  dst_vec << std::endl << std::endl;
  std::cout << "Homography:  " << endl <<  H << std::endl << std::endl;
  std::vector<Mat> rotations;
  std::vector<Mat> translations;
  std::vector<Mat> normals;
  Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity(3,3);
  Mat K_mat(3,3, CV_8U);
  //cv2eigen(K_mat, K_);
  eigen2cv(K_, K_mat);

  decomposeHomographyMat(H, K_mat, rotations, translations, normals);
  Mat R;
  Mat T;
  /*
  if (rotations.size() == 1){
    R = rotations.at(0);
    T = translations.at(0);
    std::cout << "R :" << std::endl << R << std::endl << std::endl;
  }
  */
  if (rotations.size() == 4){
    std::tuple<Mat, Mat> transformation = transformation_from_homography(rotations, translations, normals, board);
    R = std::get<0>(transformation);
    T = std::get<1>(transformation);

    //std::cout << rotations.size() << "," << translations.size() << std::endl;
  }
  else {
    ROS_ERROR("NOT FOUR POSSIBLE ROTATIONS");
    std::cout << "rotations: " << std::endl << rotations.size() << std::endl << std::endl;

    return std::make_tuple(false,G);
  }

  G(0,0) = R.at<double>(0,0);
  G(0,1) = R.at<double>(0,1);
  G(0,2) = R.at<double>(0,2);
  G(0,3) = T.at<double>(0,0);
  G(1,0) = R.at<double>(1,0);
  G(1,1) = R.at<double>(1,1);
  G(1,2) = R.at<double>(1,2);
  G(1,3) = T.at<double>(1,0);
  G(2,0) = R.at<double>(2,0);
  G(2,1) = R.at<double>(2,1);
  G(2,2) = R.at<double>(2,2);
  G(2,3) = T.at<double>(2,0);
  G(3,3) = 1.0;
  return std::make_tuple(true, G);

}

std::tuple<Mat, Mat> ImageConverter::transformation_from_homography(
              std::vector<Mat> rotations, std::vector<Mat> translations,
                                              std::vector<Mat> normals, int board){

    Mat R;
    Mat T;
    assert(rotations.size() == 4);
    int transformation_num = 0;
    float max_n1 = -1.0;
    float max_n2 = -1.0;
    //float min_n = 1.0;
    for (int i=0; i<4; i++){
      Mat r = rotations.at(i);
      Mat t = translations.at(i);
      Mat n = normals.at(i);
      Eigen::MatrixXd R_;
      Eigen::VectorXd T_;
      cv2eigen(r, R_);
      cv2eigen(t, T_);

      if (board == 1){
        if (t.at<double>(2,0) > 0){
          //We want the dot product between [0,0,1]^T and 'n' to equal 0
          //Choose largest smallest third n value
          if (n.at<double>(2,0) > max_n1){
            std::cout << "n: " << std::endl << n << std::endl << std::endl;
            std::cout << "t: " << std::endl << t << std::endl << std::endl;
            std::cout << "R: " << std::endl << r << std::endl << std::endl;
            std::cout << "R*t: " << R_*T_ << std::endl << std::endl;
            max_n1 = n.at<double>(2,0);
            //std::cout << "max_n: " << max_n << std::endl << std::endl;
            transformation_num = i;
          }
        }
      }
      else if(board == 2){
        if (t.at<double>(2,0) < 0){
          if (n.at<double>(2,0) > max_n2){
            std::cout << "n: " << std::endl << n << std::endl << std::endl;
            std::cout << "t: " << std::endl << t << std::endl << std::endl;
            std::cout << "R: " << std::endl << r << std::endl << std::endl;
            std::cout << "R*t: " << R_*T_ << std::endl << std::endl;
            max_n2 = n.at<double>(2,0);
            std::cout << "max_n: " << max_n2 << std::endl;
            transformation_num = i;
          }
        }
      }
    }
    R = rotations.at(transformation_num);
    T = translations.at(transformation_num);

    return std::make_tuple(R,T);
}


void ImageConverter::static_frame_transformation(){
  /*****************************************************************************
  Determine a scaled rotation and translation between frames from one static frame
  *****************************************************************************/
  //Find checkerboard points in image frame
  std::tuple<bool, Eigen::MatrixXd, Eigen::MatrixXd> chess_points = chess_board_points(cv_ptr);
  bool found = std::get<0>(chess_points);
  Eigen::MatrixXd Xi = std::get<1>(chess_points); //Image plane homogenous points
  Eigen::MatrixXd Xw = std::get<2>(chess_points); //World plane homogenous points
  //Handle some common problems
  if (!found){
    ROS_ERROR("CHECKERBOARD NOT VISIBLE");
    return ;
  }

  if (Xd.cols() != 4){
    std::cout << Xd << std::endl;
    ROS_ERROR("MUST SELCT THE FOUR CHECKERBOARD CORNERS");
    return ;
  }
  /***********************************
  Color camera to plane transformation
  ***********************************/
  std::vector<Point2f> Xi_vec = matrix_to_vector(Xi);
  std::vector<Point2f> Xw_vec = matrix_to_vector(Xw);

  //Get transformation
  std::tuple<bool, Eigen::MatrixXd> color_camera = planar_transformation(Xi_vec, Xw_vec, 1);
  bool good_color_transform = std::get<0>(color_camera);
  Eigen::Matrix4d color_camera_transformation = std::get<1>(color_camera);
  std::cout << "camera trans: " << std::endl << color_camera_transformation << std::endl << std::endl;
  /***********************************
  TOF to plane transformation
  ***********************************/
  std::vector<Point2f> Xd_vec = matrix_to_vector(Xd);
  //std::cout << "xd_vec" << Xd_vec << std::endl;
  std::vector<Point2f> Xv_vec = matrix_to_vector(Xv);
  //std::cout << "xv_vec" << Xv_vec << std::endl;
  //Get transformation
  std::tuple<bool, Eigen::MatrixXd> depth_camera = planar_transformation(Xd_vec, Xv_vec, 2);
  bool good_depth_transform = std::get<0>(depth_camera);
  Eigen::Matrix4d depth_camera_transformation = std::get<1>(depth_camera);
  //Eigen::MatrixXd Xv_pinv = Xv.completeOrthogonalDecomposition().pseudoInverse();
  //Eigen::MatrixXd depth_camera_transformation = Xd*Xv_pinv;
  std::cout << "depth trans: " << std::endl << depth_camera_transformation << std::endl << std::endl;

  //Record transformation
  if(good_color_transform && good_depth_transform){
    //int age;
    //cin >> age;
    //std::cout << age << std::endl;
    camera_transformation_vector.push_back(color_camera_transformation);
    depth_transformation_vector.push_back(depth_camera_transformation);
    ROS_INFO("POINTS RECORDED");
  }
  else{
    ROS_INFO("BAD POINTS, IGNORED");
  }
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
    //std::cout << "Gc: " << std::endl << Gc << std::endl << std::endl;
    //Isolate 't' vectors
    Eigen::Vector3d t_c(Gc(0,3), Gc(1,3), Gc(2,3));
    Eigen::Vector3d t_d(Gd(0,3), Gd(1,3), Gd(2,3));
    //M matricies
    Eigen::Vector3d r_c(Gc(0,2), Gc(1,2), Gc(2,2));

    Mc.conservativeResize(3, Mc.cols()+1);
    Mc.col(i) = r_c;

    Eigen::Vector3d r_d(Gd(0,2), Gd(1,2), Gd(2,2));
    std::cout << "rd: " << std::endl << r_d << std::endl << std::endl;
    std::cout << "Md1: " << std::endl << Md << std::endl << std::endl;
    std::cout << "Mc1: " << std::endl << Mc << std::endl << std::endl;
    Md.conservativeResize(3, Md.cols()+1);
    Md.col(i) = r_d;
    //b vectors
    b_c.conservativeResize(1, b_c.cols()+1);
    b_c.col(i) = r_c.transpose()*t_c;
    b_d.conservativeResize(1, b_d.cols()+1);
    b_d.col(i) = r_d.transpose()*t_d;
  }

  //Calculate rotation and transformation
  //std::cout << "Mc: " << std::endl << Mc << std::endl;
  std::cout << "Md: " << std::endl << Md << std::endl;
  std::cout << "Mc: " << std::endl << Mc << std::endl;
  translation = (Mc*Mc.transpose()).inverse()*Mc*(b_c-b_d).transpose(); //We got translation!
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
        ROS_INFO("POINT REMOVED");
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

      else if (key_press == "l"){
        /******************************
        PRACTICE FOR DEBUGGING
        ******************************/
        Eigen::MatrixXd C = Eigen::Matrix4d::Identity(4,4);
        camera_transformation_vector.push_back(C);
        depth_transformation_vector.push_back(C);
        C(0,0) = 0;
        C(0,1) = 1;
        C(1,1) = 0;
        C(2,1) = 1;
        C(2,2) = 0;
        C(0,2) = 1;
        camera_transformation_vector.push_back(C);
        depth_transformation_vector.push_back(C);
        C(0,1) = 1;
        C(0,1) = 0;
        C(2,0) = 1;
        C(2,1) = 0;
        C(1,2) = 1;
        C(0,2) = 0;
        //camera_transformation_vector
        camera_transformation_vector.push_back(C);
        depth_transformation_vector.push_back(C);


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
