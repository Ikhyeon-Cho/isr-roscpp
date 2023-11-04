/*
 * subscriver_demo_node.cpp
 *
 *  Created on: Nov 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "isr_roscpp_node/node.h"
#include <std_msgs/String.h>

namespace isr
{
class SubscriberDemoNode : public roscpp::Node
{
public:
  SubscriberDemoNode();
  void nodeRegistration();
  void ChatterCallback(const std_msgs::String::ConstPtr& msg);

private:
  roscpp::Subscriber<std_msgs::String> chatter_sub;
};

SubscriberDemoNode::SubscriberDemoNode() : Node(), chatter_sub("/chatter", 1000)
{
  nodeRegistration();
}

void SubscriberDemoNode::nodeRegistration()
{
  chatter_sub.registerSubscriber(&SubscriberDemoNode::ChatterCallback, this);
}

void SubscriberDemoNode::ChatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
}  // namespace isr

int main(int argc, char** argv)
{
  ros::init(argc, argv, "isr_roscpp_subscriber_demo");
  ros::NodeHandle nh("~");

  isr::SubscriberDemoNode node;

  ros::spin();
  
  return 0;
}