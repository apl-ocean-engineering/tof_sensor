#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Float64.h"
#include <tf/transform_listener.h>
#include <math.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <fstream>
#include <string>
#include <typeinfo>
#include <vector>
#include <iostream>

class PointCloudToImage
{
public:
  //PointCloud2ConstPtr&
  //sensor_msgs::PointCloud2 p;
  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud){
  //cloud_cb (const boost::make_shared<sensor_msgs::PointCloud2>(p) cloud)
  //cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)

    if ((cloud->width * cloud->height) == 0)
      return; //return if the cloud is not dense!
    try
    {
      pcl::toROSMsg (*cloud, image_); //convert the cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }
    float width = (float) image_.width;
    float height = (float) image_.width;
    std::cout << "1: " << image_.width + image_.height << std::endl;
    image_.width = width/2.0+1;
    image_.height = height/2.0;
    std::cout << "2: " << image_.width + image_.height << std::endl;
    image_pub_.publish (image_); //publish our cloud image

  }
  PointCloudToImage () : cloud_topic_("tof_pointcloud"),image_topic_("projected_image")
  {
    sub_ = nh_.subscribe (cloud_topic_, 30,
                          &PointCloudToImage::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);

    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
  }
private:
  ros::NodeHandle nh_;
  sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher image_pub_; //image message publisher
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "convert_pointcloud_to_image");
  PointCloudToImage pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}
