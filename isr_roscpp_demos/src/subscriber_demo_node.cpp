/*
 * subscriber_demo_node.cpp
 *
 *  Created on: Nov 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "node/subscriber_demo.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "isr_roscpp_subscriber_demo");
  ros::NodeHandle nh("~");

  isr::SubscriberDemoNode node;

  ros::spin();

  return 0;
}