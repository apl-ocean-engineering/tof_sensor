# tof_sensor
ROS package to produce TOF 3D color point clouds utilizing TI's [voxelsdk](https://github.com/3dtof/voxelsdk) package and an additional monocular camera.

## Requirments
ROS (tested on Melodic)  
[voxelsdk](https://github.com/3dtof/voxelsdk)  
[Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)  
[OpenCv](https://opencv.org/)

## Installing
Clone into <catkin_ws>/src and compile the workspace using the flag -DCMAKE_BUILD_TYPE=Release  

## TODO
Need to calculate transformation G between camera and TOF frame.  
Generalize system, more to launch files.  
