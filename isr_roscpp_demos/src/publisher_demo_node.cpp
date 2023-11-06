/*
 * publisher_demo_node.cpp
 *
 *  Created on: Nov 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "node/publisher_demo.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "isr_roscpp_publisher_demo");
  ros::NodeHandle nh("~");

  isr::PublisherDemoNode node;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}