
/*
 Backtrack color information from TOF pointcloud and camera frame

 12/21/2018
 author: mitchell scott
*/
#include "color_pointcloud.h"
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

unsigned long createRGB(int r, int g, int b)
{
    return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
}

int main(int argc, char **argv){
  //Setupd ROS
  ros::init(argc, argv, "colorPointcloud", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  ImageColor imgColor = ImageColor(argc, argv);
  ros::Publisher pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("color_pointcloud", 1);
  //Loop at 5Hz
  ros::Rate rate(50);
  while (ros::ok()){
    //Get color image data
    std::vector<PointT> pointVector = imgColor.processing();
    //Re-organize
    sensor_msgs::PointCloud pointcloud;
    std_msgs::Header header;
    header.frame_id = "map";
    pointcloud.header = header;
    std::vector<geometry_msgs::Point32> points;

    int count = 0;
    std::vector<float> colorData;
    for(auto it=pointVector.begin(); it!=pointVector.end(); ++it){
      if (count < 500){
        geometry_msgs::Point32 colorPoint;
        std::vector<std::uint8_t> color;
        colorPoint.x = it->x;
        colorPoint.y = it->y;
        colorPoint.z = it->z;
        int r = (int) it->r;
        int g = (int) it->g;
        int b = (int) it->b;
        unsigned long hex = createRGB(b,g,r);
        colorData.push_back(hex);
        points.push_back(colorPoint);
        count ++;
      }
    }

    pointcloud.points = points;
    sensor_msgs::ChannelFloat32 intensityChannelMsg;
    intensityChannelMsg.name = "rgb";
    intensityChannelMsg.values = colorData;
    std::vector<sensor_msgs::ChannelFloat32> intensityChannelVector;
    intensityChannelVector.push_back(intensityChannelMsg);
    pointcloud.channels = intensityChannelVector;
    //Publish data... causes segmentation fault
    pointcloud_pub.publish(pointcloud);


    ros::spinOnce();
    rate.sleep();
  }
  return 1;
}
