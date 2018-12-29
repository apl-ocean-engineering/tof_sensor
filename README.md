# tof_sensor
ROS package to produce TOF 3D color point clouds utilizing TI's [voxelsdk](https://github.com/3dtof/voxelsdk) package and an additional monocular camera. Not yet fully functioning.  

## TODO
There is an issue with constantly occuring segmentation faults occuring when trying to publish pointclouds. Need to solve this.  
Color data is not yet accurate... how should this data be transmitted over ROS?  
Need to calculate transformation G between camera and TOF frame.  
