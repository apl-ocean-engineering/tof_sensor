cmake_minimum_required(VERSION 2.8.3)
project(tof_sensor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  genmsg
)
find_package(OpenCV REQUIRED)
find_package(Eigen 3 QUIET)
if(NOT Eigen_FOUND)
  include(${PROJ_SOURCE_DIR}/cfg/FindEigen3.cmake)
  set(Eigen_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
endif()
include_directories(${Eigen_INCLUDE_DIR})
FIND_PACKAGE(Voxel 0.1.0 REQUIRED)
FIND_PACKAGE(PCL REQUIRED)
find_package(cmake_modules REQUIRED)

add_message_files(
  FILES
  ColorPointCloud.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES tof_sensor
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs sensor_msgs
  pcl_ros pcl
  DEPENDS Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#Definitions
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#Executables
add_executable(pointcloud_image_fusion src/pointcloud_image_fusion.cpp ${catkin_LIBRARIES})
add_executable(tof_pointcloud_publisher src/TOF_pointcloud_publisher.cpp ${catkin_LIBRARIES})

## Specify libraries to link a library or executable target against
target_link_libraries(pointcloud_image_fusion ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(tof_pointcloud_publisher ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} ${VOXEL_LIBRARIES})


TARGET_INCLUDE_DIRECTORIES(tof_pointcloud_publisher PUBLIC ${VOXEL_INCLUDE_DIRS})
