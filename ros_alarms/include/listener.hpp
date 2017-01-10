#pragma once

#include <ros/ros.h>
#include <ros_alarms/Alarms.h>
#include <ros_alarms/AlarmGet.h>

#include <alarm_proxy.hpp>

#include <string>
#include <functional>

template <typename callable_t = std::function< void(ros_alarms::Alarm) >>
struct ListenerCb
{
  callable_t  cb_func;
  int severity_lo = 0;
  int severity_hi = 0;
  bool call_on_raise = true;

  bool severity_check(int severity)
    { return severity >= severity_lo && severity <= severity_hi; }

  void operator()(ros_alarms::Alarm msg)
  {
    if(call_on_raise == msg.raised && severity_check(msg.severity))
    {
      AlarmProxy alarm { msg };
      cb_func(alarm);
    }    
  }
};

template <typename callable_t = std::function< void(ros_alarms::Alarm) >>
class AlarmListener
{

public:
  AlarmListener(ros::NodeHandle &nh, std::string alarm_name);
  bool is_raised() const      { return __last_alarm.raised; }
  bool is_cleared() const     { return !is_raised(); }
  AlarmProxy get_alarm();
  void add_cb(callable_t cb, int severity, bool call_on_raise);
  void add_cb(callable_t cb, int severity_lo, int severity_hi, bool call_on_raise);
  void clear_callbacks()      { __callbacks.clear(); }

private:
  ros::NodeHandle __nh;
  std::string __alarm_name;
  ros::ServiceClient __get_alarm;
  ros::Subscriber __update_subscriber;
  std::vector<ListenerCb<callable_t>> __callbacks;
  AlarmProxy __last_alarm;
  void __alarm_update(ros_alarms::Alarms);
};

template <typename callable_t>
AlarmListener<callable_t>::AlarmListener(ros::NodeHandle &nh, std::string alarm_name)
: __nh(nh), __alarm_name(alarm_name)
{
  __get_alarm = __nh.serviceClient<ros_alarms::AlarmGet>("/alarm/get");
  __update_subscriber = __nh.subscribe("/alarm/updates", this, __alarm_update);
}

template <typename callable_t>
AlarmProxy AlarmListener<callable_t>::get_alarm()
{
  ros_alarms::AlarmGetRequest alarm_query;
  alarm_query.alarm_name = __alarm_name;
  __get_alarm.call(alarm_query);
}

template <typename callable_t>
void AlarmListener<callable_t>::add_cb(callable_t cb, int severity, bool call_on_raise)
{
  add_cb(cb, severity, severity, call_on_raise);
}

template <typename callable_t>
void AlarmListener<callable_t>::add_cb(callable_t cb, int s_lo, int s_hi, bool call_on_raise)
{
  ListenerCb<callable_t> l_cb;
  l_cb.callback = cb;
  l_cb.severity_lo = s_lo;
  l_cb.severity_hi = s_hi;
  l_cb.call_on_raise = call_on_raise;

  __callbacks.push_back(l_cb);
}

template <typename callable_t>
void AlarmListener<callable_t>::__alarm_update(ros_alarms::Alarms alarms)
{
  for(auto alarm_msg : alarms.alarms)
  {
    if(alarm_msg.alarm_name == __last_alarm.alarm_name)
    {
      __last_alarm = alarm_msg;
      
      for(auto& cb : __callbacks)
        cb(alarm_msg);
    }
  }
}
