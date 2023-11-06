cmake_minimum_required(VERSION 3.0.2)
project(isr_roscpp_demos)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  isr_roscpp_node
)

catkin_package(
 INCLUDE_DIRS 
    include
 CATKIN_DEPENDS
    isr_roscpp_node
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(publisher_demo_node src/publisher_demo_node.cpp
  src/node/publisher_demo.cpp
)
target_link_libraries(publisher_demo_node
  ${catkin_LIBRARIES}
)

add_executable(subscriber_demo_node src/subscriber_demo_node.cpp
  src/node/subscriber_demo.cpp
)
target_link_libraries(subscriber_demo_node
  ${catkin_LIBRARIES}
)