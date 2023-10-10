cmake_minimum_required(VERSION 3.0.2)
project(isr_roscpp_tools)

add_compile_options(-std=c++14)

find_package(PCL 1.10 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ddynamic_reconfigure
  tf2_ros
  pcl_ros
)

catkin_package(
 INCLUDE_DIRS 
    include
 CATKIN_DEPENDS 
    roscpp 
    ddynamic_reconfigure 
    tf2_ros 
    pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)