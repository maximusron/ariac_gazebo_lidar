cmake_minimum_required(VERSION 3.0.2)
add_compile_options(-O3)
project(pointcloud_clustering)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
   INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch config include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

add_executable(pointcloud_clustering src/pcl_cluster_obj.cpp)
target_link_libraries(pointcloud_clustering ${catkin_LIBRARIES})