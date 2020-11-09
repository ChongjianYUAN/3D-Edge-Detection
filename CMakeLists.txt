cmake_minimum_required(VERSION 2.8.3)
project(unity_sw_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  livox_ros_driver
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include/sophus
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS include
)

add_executable(unity_feature src/unity_feature.cpp)
target_link_libraries(unity_feature ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(unity_mapping src/unity_mapping.cpp)
target_link_libraries(unity_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
