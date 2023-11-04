cmake_minimum_required(VERSION 3.0.2)
project(isr_roscpp_node)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  isr_roscpp_core
  isr_roscpp_tools
)

catkin_package(
 INCLUDE_DIRS 
    include
 CATKIN_DEPENDS 
    isr_roscpp_core
    isr_roscpp_tools
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)