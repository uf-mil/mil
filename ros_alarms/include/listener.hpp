#pragma once

#include <ros/ros.h>
#include <ros_alarms/Alarms.h>
#include <ros_alarms/AlarmGet.h>

#include <alarm_proxy.hpp>

#include <string>
#include <functional>

template <typename T>
class AlarmListener
{
public:
  AlarmListener(ros::NodeHandle &nh, std::string alarm_name);
  bool is_raised() const;
  bool is_cleared() const;
  AlarmProxy getAlarm() const;
  void add_cb(T, int severity);
  void add_cb(T, int severity_lo, int severity_hi);
  void clear_callbacks();
private:
  ros::NodeHandle __nh;
  ros::ServiceClient __get_alarm;
  std::vector<T> __callbacks;
  AlarmProxy last_alarm;
  bool __cb_severity_check(int severity);
  bool __cb_severity_check(int severity_lo, int severity_hi);
  void __alarm_update(ros_alarms::Alarms);
};

template <typename T>
AlarmListener<T>::AlarmListener(ros::NodeHandle &nh, std::string alarm_name)
{

}
