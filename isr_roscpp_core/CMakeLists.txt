cmake_minimum_required(VERSION 3.0.2)
project(isr_roscpp_core)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
 INCLUDE_DIRS 
    include
 CATKIN_DEPENDS 
    roscpp 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)