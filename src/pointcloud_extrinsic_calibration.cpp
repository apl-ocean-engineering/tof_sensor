#include "pointcloud_extrinsic_calibration.h"

PointcloudExtrinsic::PointcloudExtrinsic(){

}

PointcloudExtrinsic::~PointcloudExtrinsic(){

}


void PointcloudExtrinsic::PointcloudLeftCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::fromROSMsg(*msg, cloudLeft);
}


void PointcloudExtrinsic::PointcloudRigtCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::fromROSMsg(*msg, cloudRight);
}

void PointcloudExtrinsic::points3D_callbackLeft(
    const pcl::visualization::PointPickingEvent &event, void *) {
  /*********************************************************
  PCL callback. Add PCL point click to Xd point click matrix
  *********************************************************/
  float x, y, z;
  if (event.getPointIndex() != -1) {
    // If clicked, get point
    event.getPoint(x, y, z);
    ROS_INFO("Left PointCloud point selected: ");
    std::cout << "(x: " << x << " y: " << y << " z: " << z << ")" << std::endl;
    ROS_INFO("Press r to remove the last point");
    // Add point to Xd matrix
    leftPc_ponts.conservativeResize(4, leftPc_ponts.cols() + 1);
    Eigen::Vector4d x_(x, y, z, 1.0);
    leftPc_ponts.col(leftPc_ponts.cols() - 1) = x_;
  }
}

void PointcloudExtrinsic::points3D_callbackRight(
    const pcl::visualization::PointPickingEvent &event, void *) {
  /*********************************************************
  PCL callback. Add PCL point click to Xd point click matrix
  *********************************************************/
  float x, y, z;
  if (event.getPointIndex() != -1) {
    // If clicked, get point
    event.getPoint(x, y, z);
    ROS_INFO("Right PointCloud point selected: ");
    std::cout << "(x: " << x << " y: " << y << " z: " << z << ")" << std::endl;
    ROS_INFO("Press r to remove the last point");
    // Add point to Xd matrix
    rightPc_ponts.conservativeResize(4, rightPc_ponts.cols() + 1);
    Eigen::Vector4d x_(x, y, z, 1.0);
    rightPc_ponts.col(leftPc_ponts.cols() - 1) = x_;
  }
}

void PointcloudExtrinsic::keyboard_callback(
    const pcl::visualization::KeyboardEvent &event, void *) {
  /*******************************
  PCL callback. Get keyboard click
  *******************************/
  if (event.getKeyCode() && event.keyDown()) {
    // If pressend, get which key was pressed
    key_press = event.getKeyCode();
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> PointcloudExtrinsic::createViewer(int PC)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));

  // Set basic cloud properties
  viewer->setBackgroundColor(0, 0, 0);
  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  // viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  viewer->setCameraPosition(-0.0653346, -0.449155, -0.389522, -0.3796,
                            -0.123545, -1.08106, 0.00989458, 0.90641, 0.422284);

  // Register the PCL callbacks we need
  if (PC == 1)
  {
    viewer->registerPointPickingCallback(&PointcloudExtrinsic::points3D_callbackLeft,
                                         *this);
    viewer->registerKeyboardCallback(&PointcloudExtrinsic::keyboard_callback,
                                     *this);
  }

  if (PC == 2)
  {
    viewer->registerPointPickingCallback(&PointcloudExtrinsic::points3D_callbackRight,
                                         *this);
  }


  return viewer;
}

void PointcloudExtrinsic::run(){
  PointCloudT::Ptr cloud_left_ptr(new PointCloudT);
  PointCloudT::Ptr cloud_right_ptr(new PointCloudT);
  //Setup PCL viewers
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerLeft = createViewer(1);
  viewerLeft->addPointCloud<pcl::PointXYZ>(cloud_left_ptr, "sample cloud");
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerRight = createViewer(2);
  viewerLeft->addPointCloud<pcl::PointXYZ>(cloud_left_ptr, "sample cloud");
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    *cloud_left_ptr = cloudLeft;
    *cloud_right_ptr = cloudRight;

    if (key_press == "r") {
      leftPc_ponts.conservativeResize(leftPc_ponts.rows(), leftPc_ponts.cols() - 1);
      rightPc_ponts.conservativeResize(rightPc_ponts.rows(), rightPc_ponts.cols() - 1);
      ROS_INFO("POINT REMOVED");
      key_press = " ";
    }

    if (key_press == "f") {
      Eigen::Matrix4d G = leftPc_ponts*rightPc_ponts.completeOrthogonalDecomposition().pseudoInverse();
      std::cout << G << std::endl;

    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}


int main(int argc, char **argv){
  ros::init(argc, argv, "pointloud_extrinsic");
  ros::NodeHandle nh_("pointcloud_etrinsic");


  std::string pointcloud_topic1, pointcloud_topic2;

  if (nh_.getParam("left_pointcloud", pointcloud_topic1)){
    ROS_INFO("Left pointcloud subscribed: %s", pointcloud_topic1.c_str());
  }
  else{
    ROS_WARN("No left pointcloud topic specified");
    pointcloud_topic1 = "/camera/points2";
  }

  if (nh_.getParam("right_pointcloud", pointcloud_topic2)){
    ROS_INFO("Left pointcloud subscribed: %s", pointcloud_topic2.c_str());
  }
  else{
    ROS_WARN("No left pointcloud topic specified");
    pointcloud_topic2 = "/seikowave/points";
  }

  PointcloudExtrinsic pe;

  ros::Subscriber leftPointcloudSub = nh_.subscribe(pointcloud_topic1, 100,
    &PointcloudExtrinsic::PointcloudLeftCallback, &pe);
  ros::Subscriber rightPointcloudSub = nh_.subscribe(pointcloud_topic2, 100,
    &PointcloudExtrinsic::PointcloudRigtCallback, &pe);

  pe.run();


}
