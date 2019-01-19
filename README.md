# tof_sensor
ROS package to produce TOF 3D color point clouds utilizing TI's [voxelsdk](https://github.com/3dtof/voxelsdk) package and an additional monocular camera.  

Very much a work in progress... not intended for external use.

## Requirments
ROS (tested on Melodic)  
[voxelsdk](https://github.com/3dtof/voxelsdk)  
[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)  
[OpenCv](https://opencv.org/)

## Installing
Clone into <catkin_ws>/src and compile the workspace using the flag -DCMAKE_BUILD_TYPE=Release   

## Run
There are three main nodes in the package so far: 1) tof_pointcloud_publisher, which publishes pointcloud data via. ROS from a TOF sensor using TI's VoxelSDK package. 2) extrinsic_calibration, which will extrinsically calbirate a TOF-Visible light sensor. 3) pointcloud_image_fusion, which will fuse a calibrated pointcloud with associated RGB image (this code is a bit rough as I debug some math).   

## Development Issues:
1. Should probably put more care into garanting pointclouds and images are from the same timestamp.  
2. There is a weird lag/reduced frequency problem that sometimes occurs when recording via. ROS bag... not yet sure the issue.  
3. All points being transformed by 'G' are being mapped to [0,0,0]^T at some timestamps... this makes no sense mathmatically, and is likley an issue with some upstream processing.   
