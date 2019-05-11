#include "pointcloud_extrinsic_calibration.h"

PointcloudExtrinsic::PointcloudExtrinsic() {
  update_left = false;
  update_right = false;
}

PointcloudExtrinsic::~PointcloudExtrinsic() {}

void PointcloudExtrinsic::PointcloudLeftCallback(
    const sensor_msgs::PointCloud2ConstPtr &msg) {
  // ROS_INFO("LEFT");
  pcl::fromROSMsg(*msg, cloudLeft);
  update_left = true;
}

void PointcloudExtrinsic::PointcloudRigtCallback(
    const sensor_msgs::PointCloud2ConstPtr &msg) {
  // ROS_INFO("RIGHT");
  pcl::fromROSMsg(*msg, cloudRight);
  update_right = true;
}

void PointcloudExtrinsic::points3D_callbackLeft(
    const pcl::visualization::PointPickingEvent &event, void *) {
  /*********************************************************
  PCL callback. Add PCL point click to Xf point click matrix
  *********************************************************/
  float x, y, z;
  if (event.getPointIndex() != -1) {
    // If clicked, get point
    event.getPoint(x, y, z);
    ROS_INFO("Left PointCloud point selected: ");
    std::cout << "(x: " << x << " y: " << y << " z: " << z << ")" << std::endl;
    ROS_INFO("Press r to remove the last point");
    // Add point to Xf matrix
    leftPc_points.conservativeResize(4, leftPc_points.cols() + 1);
    Eigen::Vector4f x_(x, y, z, 1.0);
    leftPc_points.col(leftPc_points.cols() - 1) = x_;
  }
  std::cout << "Left Point size: " << leftPc_points.cols() << std::endl;
}

void PointcloudExtrinsic::points3D_callbackRight(
    const pcl::visualization::PointPickingEvent &event, void *) {
  /*********************************************************
  PCL callback. Add PCL point click to Xf point click matrix
  *********************************************************/
  float x, y, z;
  if (event.getPointIndex() != -1) {
    // If clicked, get point
    event.getPoint(x, y, z);
    ROS_INFO("Right PointCloud point selected: ");
    std::cout << "(x: " << x << " y: " << y << " z: " << z << ")" << std::endl;
    ROS_INFO("Press r to remove the last point");
    // Add point to Xf matrix
    rightPc_points.conservativeResize(4, rightPc_points.cols() + 1);
    Eigen::Vector4f x_(x, y, z, 1.0);
    rightPc_points.col(rightPc_points.cols() - 1) = x_;
  }
  std::cout << "Right Point size: " << rightPc_points.cols() << std::endl;
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

boost::shared_ptr<pcl::visualization::PCLVisualizer>
PointcloudExtrinsic::createViewer(std::string name,
                                  PointCloudT::Ptr inputCloud) {
  std::string visualization_name = name + " viewer";
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer(visualization_name));

  // Set basic cloud properties
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(inputCloud, name);

  viewer->setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  /*
  // Register the PCL callbacks we need
  if (name == "left camera") {
    viewer->registerPointPickingCallback(
        &PointcloudExtrinsic::points3D_callbackLeft, *this);
    viewer->registerKeyboardCallback(&PointcloudExtrinsic::keyboard_callback,
                                     *this);
  }

  if (name == "right camera") {
    viewer->registerPointPickingCallback(
        &PointcloudExtrinsic::points3D_callbackRight, *this);
  }
  */

  return viewer;
}

void PointcloudExtrinsic::run() {
  PointCloudT::Ptr cloud_left_ptr(new PointCloudT);
  PointCloudT::Ptr cloud_right_ptr(new PointCloudT);
  PointCloudT::Ptr cloud_left_filtered(new PointCloudT);
  PointCloudT::Ptr cloud_right_filtered(new PointCloudT);
  // Setup PCL viewers
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerLeft =
      createViewer("left cloud", cloud_left_ptr);

  // viewerLeft->setCameraPosition(-0.231269, -0.183611, -1.09393, -0.269724,
  //                               -0.220438, -1.91865, 0.181527, -0.982748,
  //                               0.0354193);
  viewerLeft->registerPointPickingCallback(
      &PointcloudExtrinsic::points3D_callbackLeft, *this);
  viewerLeft->registerKeyboardCallback(&PointcloudExtrinsic::keyboard_callback,
                                       *this);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerRight =
      createViewer("right cloud", cloud_right_ptr);
  viewerRight->registerPointPickingCallback(
      &PointcloudExtrinsic::points3D_callbackRight, *this);
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    *cloud_left_ptr = cloudLeft;
    *cloud_right_ptr = cloudRight;

    if (!viewerLeft->wasStopped()) {
      if (update_left) {
        pcl::PassThrough<pcl::PointXYZ> pass;
        float z_min = 0.0;
        float z_max = 2.0;
        pass.setInputCloud(cloud_left_ptr);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(z_min, z_max);
        pass.filter(*cloud_left_filtered);
        viewerLeft->updatePointCloud<PointT>(cloud_left_filtered, "left cloud");
        update_left = false;
      }
      if (update_right) {

        pcl::RandomSample<pcl::PointXYZ> random_sample;
        random_sample.setInputCloud(cloud_right_ptr);
        random_sample.setSample(cloud_right_ptr->size() / 2);
        random_sample.filter(*cloud_right_filtered);

        // pcl::PassThrough<pcl::PointXYZ> pass;
        // float z_min = 0.0;
        // float z_max = 2.0;
        // pass.setInputCloud(cloud_right_ptr);
        // pass.setFilterFieldName("z");
        // pass.setFilterLimits(z_min, z_max);
        // pass.filter(*cloud_right_filtered);
        //
        // PointCloudT::Ptr sensorCloudTransform(new PointCloudT);
        // Eigen::Matrix4f Rx = Eigen::Matrix4f::Zero();
        // Rx(0, 0) = 1.0;
        // Rx(1, 1) = -1.0;
        // Rx(2, 2) = -1.0;
        // Rx(3, 3) = 1.0;
        // pcl::transformPointCloud(*cloud_right_filtered,
        // *sensorCloudTransform,
        //                          Rx);

        viewerRight->updatePointCloud<PointT>(cloud_right_filtered,
                                              "right cloud");

        update_right = false;
      }
    }

    if (key_press == "r") {
      leftPc_points.conservativeResize(leftPc_points.rows(),
                                       leftPc_points.cols() - 1);
      rightPc_points.conservativeResize(rightPc_points.rows(),
                                        rightPc_points.cols() - 1);
      std::cout << "Left Point size: " << leftPc_points.cols() << std::endl;
      std::cout << "Right Point size: " << rightPc_points.cols() << std::endl;
      ROS_INFO("POINT REMOVED");
      key_press = " ";
    }
    // std::cout << key_press << std::endl;
    if (key_press == "f") {

      PointCloudT::Ptr leftPC(new PointCloudT);
      leftPC->points.resize(leftPc_points.cols());
      PointCloudT::Ptr rightPC(new PointCloudT);
      rightPC->points.resize(rightPc_points.cols());
      std::cout << "Left Point size: " << leftPc_points.cols() << std::endl;
      std::cout << "Right Point size: " << rightPc_points.cols() << std::endl;
      for (int i = 0; i < rightPc_points.cols(); ++i) {
        leftPC->points.at(i).x = leftPc_points.col(i)[0];
        leftPC->points.at(i).y = leftPc_points.col(i)[1];
        leftPC->points.at(i).z = leftPc_points.col(i)[2];

        rightPC->points.at(i).x = rightPc_points.col(i)[0];
        rightPC->points.at(i).y = rightPc_points.col(i)[1];
        rightPC->points.at(i).z = rightPc_points.col(i)[2];
      }
      pcl::registration::TransformationEstimationSVD<pcl::PointXYZ,
                                                     pcl::PointXYZ>
          TESVD;
      pcl::registration::TransformationEstimationSVD<
          pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transformation2;
      TESVD.estimateRigidTransformation(*leftPC, *rightPC, transformation2);

      std::cout << transformation2 << std::endl;
      key_press = " ";
    }

    viewerLeft->spinOnce();
    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointloud_extrinsic");
  ros::NodeHandle nh_("pointcloud_etrinsic");

  std::string pointcloud_topic1, pointcloud_topic2;

  if (nh_.getParam("left_pointcloud", pointcloud_topic1)) {
    ROS_INFO("Left pointcloud subscribed: %s", pointcloud_topic1.c_str());
  } else {
    ROS_WARN(
        "No left pointcloud topic specified, subscribing to /camera/points2");
    pointcloud_topic1 = "/camera/points2";
  }

  if (nh_.getParam("right_pointcloud", pointcloud_topic2)) {
    ROS_INFO("Left pointcloud subscribed: %s", pointcloud_topic2.c_str());
  } else {
    ROS_WARN("No left pointcloud topic specified, subscribing to "
             "/seikowave_node/cloud");
    pointcloud_topic2 = "/seikowave_node/cloud";
    // pointcloud_topic2 = "/camera/points2";
  }

  PointcloudExtrinsic pe;

  ros::Subscriber leftPointcloudSub =
      nh_.subscribe(pointcloud_topic1, 100,
                    &PointcloudExtrinsic::PointcloudLeftCallback, &pe);
  ros::Subscriber rightPointcloudSub =
      nh_.subscribe(pointcloud_topic2, 100,
                    &PointcloudExtrinsic::PointcloudRigtCallback, &pe);

  pe.run();
}
