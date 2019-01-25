# tof_sensor
ROS package to produce TOF 3D color point clouds utilizing TI's [voxelsdk](https://github.com/3dtof/voxelsdk) package and an additional monocular camera. Can calculate the extrinsic paramaters of a depth sensor and a monocular camera, based on: D. Herrera C., J. Kannala and J. Heikkilä, "Joint Depth and Color Camera Calibration with Distortion Correction," in IEEE Transactions on Pattern Analysis and Machine Intelligence, vol. 34, no. 10, pp. 2058-2064, Oct. 2012.   

Still a work in progress...

## Requirments
ROS (tested on Melodic)  
[voxelsdk](https://github.com/3dtof/voxelsdk)  
[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)  
[OpenCv](https://opencv.org/)

## Installing
Clone into <catkin_ws>/src and compile using catkin_make in <catkin_ws> location.   

## Running
There are three main nodes in the package so far: 1) tof_pointcloud_publisher, which publishes pointcloud data via. ROS from a TOF sensor using TI's VoxelSDK package. 2) extrinsic_calibration, which will extrinsically calbirate a TOF-Visible light sensor. 3) pointcloud_image_fusion, which will fuse a calibrated pointcloud with associated RGB image. Not very user friendly yet.   

