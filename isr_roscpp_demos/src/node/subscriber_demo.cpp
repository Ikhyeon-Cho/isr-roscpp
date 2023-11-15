/*
 * subscriber_demo.cpp
 *
 *  Created on: Nov 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "node/subscriber_demo.h"

namespace isr
{
SubscriberDemoNode::SubscriberDemoNode() : Node(), chatter_sub("/chatter", 1000)
{
  nodeRegistration();
}

void SubscriberDemoNode::nodeRegistration()
{
  chatter_sub.registerCallback(&SubscriberDemoNode::ChatterCallback, this);
}

void SubscriberDemoNode::ChatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
}  // namespace isr