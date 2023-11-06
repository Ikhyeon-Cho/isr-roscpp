/*
 * subscriber.h
 *
 *  Created on: Sep 20, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_CORE_SUBSCRIBER_H
#define ISR_ROSCPP_CORE_SUBSCRIBER_H

#include <isr_roscpp_core/parameter.h>

namespace isr::roscpp
{
template <typename T>
class Subscriber
{
public:
  /*
   * Constructor for ROS subscriber wrapper
   * \param default_topic Default topic value for subscribing.
   * \param queue_size Msg queue size for subscribed topic.
   * **/
  Subscriber(const std::string& default_topic, uint32_t queue_size)
    : nh_("~"), topic_(default_topic), queue_size_(queue_size)
  {
  }

  /*
   * Constructor for ROS subscriber wrapper
   * \param default_topic Default topic value for subscribing.
   * **/
  Subscriber(const std::string& default_topic) : nh_("~"), topic_(default_topic)
  {
  }

  /*
   * Constructor for ROS subscriber wrapper
   * \param nh Nodehandle in the node should be passed to the class object.
   * \param default_topic Default topic value for subscribing.
   * \param queue_size Queue size for subscribed topic.
   * **/
  Subscriber(const ros::NodeHandle& nh, const std::string& default_topic, int queue_size)
    : nh_(nh), topic_(default_topic), queue_size_(queue_size)
  {
  }

  /*
   * Constructor for ROS subscriber wrapper
   * \param nh Nodehandle in the node should be passed to the class object.
   * \param default_topic Default topic value for subscribing.
   * **/
  Subscriber(const ros::NodeHandle& nh, const std::string& default_topic) : nh_(nh), topic_(default_topic)
  {
  }

  /*
   * \param param_name The key to be searched on the parameter server.
   * **/
  bool readParameter(const std::string& param_name);

  /*
   * \param param_name The key to be searched on the parameter server.
   * \param default_topic Default value to use if the server doesn't contain topic parameters
   * **/
  bool readParameter(const std::string& param_name, const std::string& default_topic);

  /*
   * \param fp Callback function for subscribed topic msgs
   * \param obj Class object for call fp function
   * \param queue_size Msg queue size for subscribed topic (Default: 10)
   * **/
  template <class M>
  void addCallback(void (M::*fp)(const boost::shared_ptr<T const>&), M* obj, uint32_t queue_size = 10);

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  Parameter<std::string> topic_;
  Parameter<int> queue_size_{ 10 };
};

template <typename T>
inline bool Subscriber<T>::readParameter(const std::string& param_name)
{
  return topic_.readParameter(param_name);
}

template <typename T>
inline bool Subscriber<T>::readParameter(const std::string& param_name, const std::string& default_topic)
{
  return topic_.readParameter(param_name, default_topic);
}

template <typename T>
template <typename M>
inline void Subscriber<T>::addCallback(void (M::*fp)(const boost::shared_ptr<T const>&), M* obj,
                                              uint32_t _queue_size)
{
  queue_size_.set(_queue_size);
  sub_ = nh_.subscribe<T>(topic_.value(), queue_size_.value(), fp, obj);
}

}  // namespace isr::roscpp
#endif  // ISR_ROSCPP_CORE_SUBSCRIBER_H