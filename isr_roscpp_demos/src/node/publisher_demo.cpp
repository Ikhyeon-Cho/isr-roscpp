/*
 * publisher_demo.cpp
 *
 *  Created on: Nov 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "node/publisher_demo.h"

namespace isr
{
PublisherDemoNode::PublisherDemoNode()
{
  nodeRegistration();

  initNodeTimers();
}

void PublisherDemoNode::nodeRegistration()
{
  chatter_pub.registerPublisher();
  publish_timer.registerCallback(&PublisherDemoNode::publishTopic, this);
}

void PublisherDemoNode::initNodeTimers()
{
  publish_timer.start();
}

void PublisherDemoNode::publishTopic(const ros::TimerEvent& event)
{
  std_msgs::String msg;
  std::stringstream ss;
  ss << "hello world " << count;
  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());
  chatter_pub.publish(msg);

  ++count;
}
}  // namespace isr