/*
 * publisher_demo_node.cpp
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
class PublisherDemoNode : public roscpp::Node
{
public:
  PublisherDemoNode();
  void nodeRegistration();
  void publishTopic(const ros::TimerEvent& event);

private:
  void initNodeTimers();

private:
  roscpp::Publisher<std_msgs::String> chatter_pub{ "/chatter", 1000 };
  roscpp::Timer publish_timer{ ros::Duration(0.1) };
  int count{ 0 };
};

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