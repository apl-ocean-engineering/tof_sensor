#include "extrinsic_calibration.h"


ExtrinsicCalibration::ExtrinsicCalibration(std::string image_topic): it_(nh_) {
    /**********************************************************
    Subscrive to input video feed and publish output video feed
    K = Eigen::Matrix3d::Zero(3, 3);
    Create board point vector, counterclockwise from bottom left
    Just estimating for now...
    **********************************************************/
    double check_x = 0.6604; //m
    double check_y = 0.8382; //m

    Xv.conservativeResize(4, Xv.cols()+1);
    Xv.col(Xv.cols()-1) =  Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
    Xv.conservativeResize(4, Xv.cols()+1);
    Xv.col(Xv.cols()-1) = Eigen::Vector4d(0.0, check_y, 0.0, 1.0);
    Xv.conservativeResize(4, Xv.cols()+1);
    Xv.col(Xv.cols()-1) = Eigen::Vector4d(check_x, check_y, 0.0, 1.0);
    Xv.conservativeResize(4, Xv.cols()+1);
    Xv.col(Xv.cols()-1) = Eigen::Vector4d(check_x, 0.0, 0.0, 1.0);

    chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    image_sub_ = it_.subscribe(image_topic, 1,
                          &ExtrinsicCalibration::imageCb, this);

    cv::namedWindow(OPENCV_WINDOW);
}

ExtrinsicCalibration::~ExtrinsicCalibration(){
    cv::destroyWindow(OPENCV_WINDOW);
}

void ExtrinsicCalibration::img_info_callback(const
                                sensor_msgs::CameraInfo::ConstPtr& info){
    /**********************************************************
    ROS callback. Turn ROS camera info message intrinsic matrix
    **********************************************************/
    fx.data = info->K[0];
    s.data = info->K[1];
    fy.data = info->K[4];
    cx.data = info->K[2];
    cy.data = info->K[5];
    //Population intrinsic matrix
    K << fx.data, s.data, cx.data, fy.data, cy.data;
    K(2,2) = 1.0;
}

void ExtrinsicCalibration::imageCb(const sensor_msgs::ImageConstPtr& msg){
    /****************************************************
    ROS callback. Turn ROS image message into OpenCv Mat
    ****************************************************/
    try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    img = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }
}

void ExtrinsicCalibration::pointcloud_callback(const
                                  sensor_msgs::PointCloud2ConstPtr& msg){
    /******************************************************
    ROS callback. Turn ROS pointcloud messages to pcl cloud
    ******************************************************/
    pcl::fromROSMsg(*msg, cloud);

}


void ExtrinsicCalibration::points3D_callback
                    (const pcl::visualization::PointPickingEvent& event, void*){
    /*********************************************************
    PCL callback. Add PCL point click to Xd point click matrix
    *********************************************************/
    float x, y, z;
    if (event.getPointIndex () != -1){
        //If clicked, get point
        event.getPoint(x, y, z);
        ROS_INFO("Point selected: ");
        std::cout << "(x: " << x << " y: " << y << " z: " << z << ")" << std::endl;
        ROS_INFO("Press r to remove the last point");
        //Add point to Xd matrix
        Xd.conservativeResize(3, Xd.cols()+1);
        Eigen::Vector3d x_(x/z, y/z, 1.0);
        Xd.col(Xd.cols()-1) =  x_;
    }
}

void ExtrinsicCalibration::keyboard_callback
                      (const pcl::visualization::KeyboardEvent& event, void* ){
    /*******************************
    PCL callback. Get keyboard click
    *******************************/
    if( event.getKeyCode() && event.keyDown()){
        //If pressend, get which key was pressed
        key_press = event.getKeyCode();
    }
}

std::vector<Point2f> ExtrinsicCalibration::matrix_to_vector(const Eigen::MatrixXd X){
  /**************************************************
  Transform Eigen Matrix to std::vector<Point2f> type
  **************************************************/
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
            ExtrinsicCalibration::chess_board_points(){
  /********************************************************
  Determine image plane coordinates of checkerboard corners
  ********************************************************/
  std::vector<cv::Point2f> checkerboard_points_vector; //Vector to record points
  cv::Mat imgGray;

  Eigen::MatrixXd Xi; //Image plane homogenous points
  Eigen::MatrixXd Xw; //World plane homogenous points

  //Determine if camera can see points
  bool found = cv::findChessboardCorners(img, patternsize,
                                  checkerboard_points_vector, chessBoardFlags);

  //Find and draw checkerboard points
  if (found){
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(imgGray, checkerboard_points_vector, cv::Size(5,5),
                cv::Size(-1,-1),
                cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT,
                30, 0.0001));
    drawChessboardCorners(img, patternsize,
                                    cv::Mat(checkerboard_points_vector), found);

    std::vector<cv::Point2f> checkerboard_points_vector_meters;

    //Transform points to meters
    for (int i=0;i<checkerboard_points_vector.size();i++){
      Point2f p;
      Eigen::Vector3d X_px(checkerboard_points_vector.at(i).x,
                                        checkerboard_points_vector.at(i).y, 1);
      Eigen::Vector3d X_m = K.inverse()*X_px;
      Eigen::Vector3d X(X_m(0), X_m(1), 1);
      Xi.conservativeResize(3, Xi.cols()+1);
      Xi.col(Xi.cols()-1) = X;
    }

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

  return std::make_tuple(found, Xi, Xw);
}

std::tuple<Mat, Mat> ExtrinsicCalibration::transformation_from_homography(
                      const std::vector<Mat> rotations, const std::vector<Mat> translations,
                                          const std::vector<Mat> normals, const int board){
    /********************************************************************
    Determine Rotation and Translation from the four Homography Solutions
    ********************************************************************/
    Mat R;
    Mat T;
    int transformation_num = 0;
    float max_n1 = -1.0;
    float max_n2 = -1.0;
    //Run through four solutions to find correct solution
    for (int i=0; i<4; i++){
      Mat r = rotations.at(i);
      Mat t = translations.at(i);
      Mat n = normals.at(i);
      Eigen::MatrixXd R_;
      Eigen::VectorXd T_;
      cv2eigen(r, R_);
      cv2eigen(t, T_);

      //Board 1 indicates camera. Different criteria for each sensor
      if (board == 1){
        /*
        Correct solution is where plane is in positive z direction and where
        the normal plane is most closly aligned with [0,0,1]^T
        */

        //Verify positive z direction
        if (t.at<double>(2,0) > 0){
          //Largest dot product between n and [0,0,1]^T occurs at max n(2)
          if (n.at<double>(2,0) > max_n1){
            max_n1 = n.at<double>(2,0);
            transformation_num = i;
          }
        }
      }

      //Board 2 indicates TOF sensor
      else if(board == 2){
        /*
        Correct solution is where plane is in negative z direction and where
        the normal plane is most closly aligned with [0,0,1]^T
        */

        //Verify negative z direction
        if (t.at<double>(2,0) < 0){
          //Largest dot product between n and [0,0,1]^T occurs at max n(2)
          if (n.at<double>(2,0) > max_n2){
            max_n2 = n.at<double>(2,0);
            transformation_num = i;
          }
        }
      }
    }

    //Get Rotation and translation from correct solution
    R = rotations.at(transformation_num);
    T = translations.at(transformation_num);

    return std::make_tuple(R,T);
}

std::tuple<bool,Eigen::Matrix4d> ExtrinsicCalibration::planar_transformation(
        const std::vector<Point2f> src_vec, const std::vector<Point2f> dst_vec, const int board){
    /*************************************************************
    Find transformaion between checkerboard plane and sensor plane
    *************************************************************/
    Eigen::Matrix4d G = Eigen::Matrix4d::Zero(4,4); //Initalize transformation

    if (src_vec.size() != dst_vec.size()){
        ROS_ERROR("CANNOT CALCULATE HOMOGRAPHY WITH UNEQUAL POINTS");
        return std::make_tuple(false,G);
    }
    //Find Homography
    Mat H = findHomography(src_vec, dst_vec);

    std::vector<Mat> rotations;
    std::vector<Mat> translations;
    std::vector<Mat> normals;
    Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity(3,3);
    Mat K_mat(3,3, CV_8U);

    //Calculate homography
    eigen2cv(K_, K_mat);
    decomposeHomographyMat(H, K_mat, rotations, translations, normals);
    Mat R;
    Mat T;

    //Determine true transformation from homography
    if (rotations.size() == 4){
        std::tuple<Mat, Mat> transformation = transformation_from_homography(
                                          rotations, translations, normals, board);
        R = std::get<0>(transformation);
        T = std::get<1>(transformation);
    }

    else {
        ROS_ERROR("NOT ENOUGH POSSIBLE ROTATIONS");
        return std::make_tuple(false, G);
    }

    //Construct G matrix from R and T. Standard SE(3) transformation matrix
    Eigen::Matrix3d R_;
    Eigen::Vector3d T_;
    cv2eigen(R,R_);
    cv2eigen(T,T_);
    G.block<3,3>(0,0) = R_;
    G.block<3,1>(2,0) = T_;
    G(3,3) = 1.0;

    return std::make_tuple(true, G);
}

void ExtrinsicCalibration::static_frame_transformation(){
    /*******************************************************************************
    Determine a scaled rotation and translation between frames from one static frame
    *******************************************************************************/
    std::tuple<bool, Eigen::MatrixXd, Eigen::MatrixXd> chess_points =
                                                    chess_board_points();
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
    //Get camera transformation
    std::tuple<bool, Eigen::MatrixXd> color_camera =
                                      planar_transformation(Xi_vec, Xw_vec, 1);
    bool good_color_transform = std::get<0>(color_camera);
    Eigen::Matrix4d color_camera_transformation = std::get<1>(color_camera);
    std::cout << "camera transformation: " << std::endl <<
                          color_camera_transformation << std::endl << std::endl;

    /***********************************
    TOF to plane transformation
    ***********************************/

    std::vector<Point2f> Xd_vec = matrix_to_vector(Xd);
    std::vector<Point2f> Xv_vec = matrix_to_vector(Xv);

    //Get depth transformation
    std::tuple<bool, Eigen::MatrixXd> depth_camera =
                                      planar_transformation(Xd_vec, Xv_vec, 2);
    bool good_depth_transform = std::get<0>(depth_camera);
    Eigen::Matrix4d depth_camera_transformation = std::get<1>(depth_camera);

    std::cout << "depth trans: " << std::endl << depth_camera_transformation
                                                    << std::endl << std::endl;

    //Record transformation
    if(good_color_transform && good_depth_transform){
        camera_transformation_vector.push_back(color_camera_transformation);
        depth_transformation_vector.push_back(depth_camera_transformation);
        ROS_INFO("POINTS RECORDED");
    }

    else{
        ROS_INFO("BAD POINTS, IGNORED");
    }

}

std::tuple<Eigen::VectorXd, Eigen::MatrixXd>
                              ExtrinsicCalibration::calculate_extrinsic_paramaters(){
    /*******************************************************************************
    Determine a scaled rotation and translation between frames from one static frame
    *******************************************************************************/

    Eigen::VectorXd translation;
    Eigen::MatrixXd rotation;

    //Verify equal transformations were preformed
    if (camera_transformation_vector.size() != depth_transformation_vector.size()){
        ROS_ERROR("UNEQUAL EXTRINSIC CALIBRATION EVENTS");
        return std::make_tuple(translation, rotation);
    }

    Eigen::MatrixXd Mc; //3XN
    Eigen::MatrixXd Md;
    Eigen::MatrixXd b_c; //1XN
    Eigen::MatrixXd b_d;

    //Populate 'M' matrix and 'b' vector to calculate transformation
    for (int i=0; i<camera_transformation_vector.size(); i++){
        //Get transformation matricies
        Eigen::MatrixXd Gc = camera_transformation_vector.at(i);
        Eigen::MatrixXd Gd = depth_transformation_vector.at(i);

        //Isolate 't' vectors
        Eigen::Vector3d t_c(Gc(0,3), Gc(1,3), Gc(2,3));
        Eigen::Vector3d t_d(Gd(0,3), Gd(1,3), Gd(2,3));

        //Append to M matricies
        Eigen::Vector3d r_c(Gc(0,2), Gc(1,2), Gc(2,2));
        Eigen::Vector3d r_d(Gd(0,2), Gd(1,2), Gd(2,2));
        Mc.conservativeResize(3, Mc.cols()+1);
        Mc.col(i) = r_c;
        Md.conservativeResize(3, Md.cols()+1);
        Md.col(i) = r_d;

        //b vectors
        b_c.conservativeResize(1, b_c.cols()+1);
        b_c.col(i) = r_c.transpose()*t_c;
        b_d.conservativeResize(1, b_d.cols()+1);
        b_d.col(i) = r_d.transpose()*t_d;
    }

    //Calculate rotation and transformation
    translation = (Mc*Mc.transpose()).inverse()*Mc*(b_c-b_d).transpose(); //We got translation!
    Eigen::MatrixXd R = Md*(Mc.transpose()); //Don't quite have R though...

    /*******************************************************************
    Need to adjust for error. Turn R orthonormal through svd. Set S == I
    *******************************************************************/
    Eigen::JacobiSVD<Eigen::MatrixXd>
                              svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
    rotation = svd.matrixU()*(svd.matrixV().transpose()); //Final rotation

    return std::make_tuple(translation, rotation);
}

void ExtrinsicCalibration::run(){

    ros::Rate loop_rate(50); //Max loop frequency

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

    //Register the PCL callbacks we need
    viewer->registerPointPickingCallback(&ExtrinsicCalibration::points3D_callback,
                                                                        *this);
    viewer->registerKeyboardCallback(&ExtrinsicCalibration::keyboard_callback,
                                                                        *this);

    //trackbar init
    //OpenCV doesn't allow for negative values :(
    namedWindow(TRACKBAR_NAME, 1);
    createTrackbar("x_min ([-1000, 1000] cm)", TRACKBAR_NAME, &x_min, 2000);
    createTrackbar("x_max ([-1000, 1000] cm)", TRACKBAR_NAME, &x_max, 2000);
    createTrackbar("y_min ([-1000, 1000] cm)", TRACKBAR_NAME, &y_min, 2000);
    createTrackbar("y_max ([-1000, 1000] cm)", TRACKBAR_NAME, &y_max, 2000);
    createTrackbar("z_min ([-1000, 1000] cm)", TRACKBAR_NAME, &z_min, 2000);
    createTrackbar("z_max ([-1000, 1000] cm)", TRACKBAR_NAME, &z_max, 2000);

    //Terminal display
    ROS_INFO("CLICK POINTS ON BOARD. PRESS 's' TO RECORD POINTS FOR 1 STATIC FRAME");
    ROS_INFO("Click clockwise from bottom left for now...");
    ROS_INFO("PRESS F TO FINISH AND CALCULATE TRANSFORMATION");

    /*****************************************************************
    Main loop to get ROS and PCL callback information and to calculate
    transformations
    *****************************************************************/
    while(ros::ok()){
        *cloud_ptr = cloud;
        //Verify pcl viewer is working
        if (!viewer->wasStopped()){
          //Get trackbar information
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

          //If r has been pushed, remove most recent entry to Xd
          if (key_press == "r"){
            Xd.conservativeResize(Xd.rows(), Xd.cols()-1);
            ROS_INFO("POINT REMOVED");
            key_press = " ";
          }


          else if (key_press == "d"){
            camera_transformation_vector.pop_back();
            depth_transformation_vector.pop_back();
            ROS_INFO("POINT REMOVED");
            key_press = " ";
          }

          //If s has been pushed, calculate [R,T] for 1 frame for both sensors
          else if (key_press == "s"){
            static_frame_transformation();
            Xd.conservativeResize(0,0);
            key_press = " ";
          }

          //If f has been pushed, calculate full transformation
          else if (key_press == "f"){
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

        //Wait and run callbacks
        waitKey(1);
        ros::spinOnce();
        viewer->spinOnce();
        loop_rate.sleep();

        }
    }
}

int main(int argc, char** argv)
{
    //Init ROS
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle n("extrinsic_calibration");

    //Get ROS caera info
    std::string camera_info;
    if (n.getParam("image_info", camera_info))
    {
    ROS_INFO("Camera info topic subscribed: %s", camera_info.c_str());
    }
    else{
    ROS_ERROR("No info topic specified, subscribing to /camera/rgb/camera_info");
    camera_info = "/camera/rgb/camera_info";
    }

    std::string image_topic;
    if (n.getParam("image", image_topic))
    {
    ROS_INFO("RGB Camera topic subscribed: %s", image_topic.c_str());
    }
    else{
    ROS_ERROR("No image topic specified, subscribing to  /camera/rgb/image_color");
    image_topic = "/camera/rgb/image_color";
    }

    std::string pointcloud;
    if (n.getParam("points", pointcloud))
    {
    ROS_INFO("RGB pointcloud topic subscribed: %s", pointcloud.c_str());
    }
    else{
    ROS_ERROR("No pointcloud topic specified, subscribed to /camera/depth/points");
    pointcloud = "/camera/depth/points";
    }

    //Create salibration obect and subscribe to ROS topics
    ExtrinsicCalibration ec(image_topic);
    ros::Subscriber info_sub = n.subscribe(camera_info, 1000,
                              &ExtrinsicCalibration::img_info_callback, &ec);
    ros::Subscriber pointcloud_sub = n.subscribe(pointcloud, 1000,
                              &ExtrinsicCalibration::pointcloud_callback, &ec);

    //Run main function
    ec.run();

    return 0;
}
