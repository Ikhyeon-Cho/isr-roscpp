/*
 * timer.h
 *
 *  Created on: Sep 20, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ISR_ROSCPP_CORE_TIMER_H
#define ISR_ROSCPP_CORE_TIMER_H

#include <isr_roscpp_core/parameter.h>

namespace isr::roscpp
{
class Timer
{
public:
  Timer(const ros::NodeHandle& nh) : nh_(nh)
  {
  }

  Timer(const ros::NodeHandle& nh, double default_duration) : nh_(nh), duration_(default_duration)
  {
  }

  Timer(const ros::NodeHandle& nh, ros::Duration default_duration) : nh_(nh), duration_(default_duration.toSec())
  {
  }

  Timer(ros::Duration default_duration) : duration_(default_duration.toSec())
  {
  }

  /*
   * \param param_name The key to be searched on the parameter server.
   * **/
  bool readParameter(const std::string& param_name)
  {
    return duration_.readParameter(param_name);
  }

  /*
   * \param param_name The key to be searched on the parameter server.
   * \param default_duration Default value to use if the server doesn't contain topic parameters
   * **/
  bool readParameter(const std::string& param_name, double default_duration)
  {
    return duration_.readParameter(param_name, default_duration);
  }

  template <class T>
  void addCallback(void (T::*fp)(const ros::TimerEvent&), T* obj, bool autostart = false, bool oneshot = false)
  {
    timer_ = nh_.createTimer(ros::Duration(duration_.value()), fp, obj, oneshot, autostart);
  }

  void start()
  {
    timer_.start();
  }

  void stop()
  {
    timer_.stop();
  }

  double getDuration() const
  {
    return duration_.value();
  }

private:
  ros::NodeHandle nh_;
  ros::Timer timer_;
  Parameter<double> duration_{ 0.1 };
};

}  // namespace isr::roscpp

#endif  // ISR_ROSCPP_CORE_TIMER_H