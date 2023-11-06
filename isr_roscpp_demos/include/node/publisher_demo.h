/*
 * publisher_demo.h
 *
 *  Created on: Nov 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_DEMOS_PUBLISHER_DEMO_H
#define ISR_ROSCPP_DEMOS_PUBLISHER_DEMO_H

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
}  // namespace isr

#endif  // ISR_ROSCPP_DEMOS_PUBLISHER_DEMO_H