/*
 * publisher.h
 *
 *  Created on: Sep 20, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_CORE_PUBLISHER_H
#define ISR_ROSCPP_CORE_PUBLISHER_H

#include <ros/publisher.h>
#include <isr_roscpp_core/parameter.h>

namespace isr::roscpp
{
template <typename T>
class Publisher
{
public:
  using Ptr = std::shared_ptr<Publisher<T>>;

  /*
   * Constructor for ROS1 publisher wrapper.
   * \param default_topic Default topic to be published
   * \param queue_size Msg queue size for publisher
   * **/
  Publisher(const std::string& default_topic, uint32_t queue_size)
    : nh_("~"), topic_(default_topic), queue_size_(queue_size)
  {
  }

  /*
   * Constructor for ROS1 publisher wrapper.
   * \param default_topic Default topic to be published
   * **/
  Publisher(const std::string& default_topic) : nh_("~"), topic_(default_topic)
  {
  }

  /*
   * Constructor for ROS1 publisher wrapper.
   * \param nh Nodehandle in the node should be passed to the class object.
   * \param default_topic Default topic value for publishing.
   * **/
  Publisher(const ros::NodeHandle& nh, const std::string& default_topic) : nh_(nh), topic_(default_topic)
  {
  }

  /*
   * Constructor for ROS1 publisher wrapper.
   * \param nh Nodehandle in the node should be passed to the class object.
   * **/
  Publisher(const ros::NodeHandle& nh) : Publisher(nh, "topic_default")
  {
  }

  /*
   * \param param_name The key to be searched on the parameter server.
   * \param _default_topic Default value to use if the server doesn't contain topic parameters
   * **/
  bool readParameter(const std::string& param_name, const std::string& default_topic);

  /*
   * \param param_name The key to be searched on the parameter server.
   * **/
  bool readParameter(const std::string& param_name);

  /*
   * \param queue_size Msg queue size for publisher
   * **/
  void registerPublisher(uint32_t queue_size);

  void registerPublisher();

  void publish(const T& msg);

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  roscpp::Parameter<std::string> topic_;

  roscpp::Parameter<int> queue_size_{ 10 };
};

template <typename T>
inline bool Publisher<T>::readParameter(const std::string& param_name, const std::string& default_topic)
{
  return topic_.readParameter(param_name, default_topic);
}

template <typename T>
inline bool Publisher<T>::readParameter(const std::string& param_name)
{
  return topic_.readParameter(param_name);
}

template <typename T>
inline void Publisher<T>::registerPublisher(uint32_t queue_size)
{
  queue_size_.set(queue_size);
  pub_ = nh_.advertise<T>(topic_.value(), queue_size_.value());
}

template <typename T>
inline void Publisher<T>::registerPublisher()
{
  pub_ = nh_.advertise<T>(topic_.value(), queue_size_.value());
}

template <typename T>
inline void Publisher<T>::publish(const T& msg)
{
  pub_.publish(msg);
}


}  // namespace isr::roscpp

#endif  // ISR_ROSCPP_CORE_PUBLISHER_H