/*
 * subscriber_demo.h
 *
 *  Created on: Nov 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_DEMOS_SUBSCRIBER_DEMO_H
#define ISR_ROSCPP_DEMOS_SUBSCRIBER_DEMO_H

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
}  // namespace isr

#endif  // ISR_ROSCPP_DEMOS_SUBSCRIBER_DEMO_H