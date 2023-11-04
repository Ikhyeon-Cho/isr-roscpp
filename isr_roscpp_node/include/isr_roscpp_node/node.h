/*
 * node.h
 *
 *  Created on: Sep 20, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_CORE_NODE_H
#define ISR_ROSCPP_CORE_NODE_H

// #include <ros/ros.h>
#include <isr_roscpp_core/publisher.h>
#include <isr_roscpp_core/subscriber.h>
#include <isr_roscpp_core/timer.h>
#include <isr_roscpp_core/parameter.h>
#include <isr_roscpp_tools/transform_handler.h>

namespace isr::roscpp
{
class Node
{
public:
  TransformHandler transform_handler{};

  std::string mapFrameId_;
  std::string baseFrameId_;
  std::string odomFrameId_;
  std::string lidarFrameId_;

private:
  ros::NodeHandle nh_;  // need this to start and shutdown the node lifetime
  ros::Time timestamp_;

public:
  Node();
  Node(const ros::NodeHandle& nh);

  virtual ~Node() = default;

  const ros::Time& getTimestamp() const
  {
    return timestamp_;
  }

  void setTimestamp(const ros::Time& _timestamp)
  {
    timestamp_ = _timestamp;
  }

  virtual void nodeRegistration() = 0;  // should be defined in the node class
};

Node::Node() : nh_("~"), timestamp_(ros::Time::now())
{
  ros::param::param<std::string>("/FrameIds/map", mapFrameId_, "map");
  ros::param::param<std::string>("/FrameIds/base", baseFrameId_, "base_link");
  ros::param::param<std::string>("/FrameIds/odom", odomFrameId_, "odom");
  ros::param::param<std::string>("/FrameIds/lidar", lidarFrameId_, "lidar");
}

Node::Node(const ros::NodeHandle& nh)
{
  setTimestamp(ros::Time::now());

  ros::param::param<std::string>("/FrameIds/map", mapFrameId_, "map");
  ros::param::param<std::string>("/FrameIds/base", baseFrameId_, "base_link");
  ros::param::param<std::string>("/FrameIds/odom", odomFrameId_, "odom");
  ros::param::param<std::string>("/FrameIds/lidar", lidarFrameId_, "lidar");
}

}  // namespace isr::roscpp

#endif  // ISR_ROSCPP_CORE_NODE_H