#pragma once

#include <ros/ros.h>
#include <ros_alarms/Alarms.h>
#include <ros_alarms/AlarmGet.h>

#include <ros_alarms/alarm_proxy.hpp>

#include <string>
#include <functional>

template <typename callable_t = std::function< void(ros_alarms::Alarm) >>
struct ListenerCb
{
  callable_t  cb_func;  // object needs to have a call operator
  int severity_lo = 0;  // highest priority
  int severity_hi = 0;  // lowest priority
  bool call_on_raise = true;

  // Compares alarm severity against the action_required range for this callback
  bool severity_check(int severity)
    { return severity >= severity_lo && severity <= severity_hi; }

  // Checks msg to see if its raised status and severity require calling
  // the associated callbacks and calls them if necessary
  void operator()(ros_alarms::Alarm msg)
  {
    bool needs_call = call_on_raise == msg.raised &&
      (call_on_raise? severity_check(msg.severity) : true);

    if(needs_call)
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
  bool is_raised() const        { return __last_alarm.raised; }
  bool is_cleared() const       { return !is_raised(); }

  // Queries server before and parses response into an AlarmProxy
  AlarmProxy get_alarm();

  // Returns AlarmProxy for last alarm of this name published to '/alarm/updates'
  AlarmProxy get_cached_alarm() { return __last_alarm; }

  // Registers a callback to be invoked at clearing or raising for a range of severities
  void add_cb(callable_t cb, int severity_lo, int severity_hi, bool call_on_raise=true);

  // Short form for registering a callback to be called for a single severity level
  void add_cb(callable_t cb, int severity, bool call_on_raise=true);

  // Registers a callback to be invoked when alarm is cleared
  void add_clear_cb(callable_t cb);

  void clear_callbacks()      { __callbacks.clear(); }

private:
  ros::NodeHandle __nh;
  std::string __alarm_name;
  ros::ServiceClient __get_alarm;
  ros::Subscriber __update_subscriber;
  std::vector<ListenerCb<callable_t>> __callbacks;
  AlarmProxy __last_alarm;
  ros::Time __last_update { 0, 0 };
  void __alarm_update(ros_alarms::Alarms);
};

template <typename callable_t>
AlarmListener<callable_t>::AlarmListener(ros::NodeHandle &nh, std::string alarm_name)
: __nh(nh), __alarm_name(alarm_name)
{
  // Service to query alarm server
  __get_alarm = __nh.serviceClient<ros_alarms::AlarmGet>("/alarm/get");

  // Subscribes to list of recently modified alarms from alarm server
  __update_subscriber = __nh.subscribe("/alarm/updates", this, __alarm_update);
}

template <typename callable_t>
AlarmProxy AlarmListener<callable_t>::get_alarm()
{
  // Create Query msg
  ros_alarms::AlarmGet alarm_query;
  alarm_query.request.alarm_name = __alarm_name;

  // Query alarm server
  if(__get_alarm.call(alarm_query))
    ROS_INFO("Alarm server query was unsuccessful.");

  // Update internal alarm data
  __last_update = alarm_query.response.header.stamp;
  __last_alarm = AlarmProxy(alarm_query.response.alarm);
  return __last_alarm;
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
void AlarmListener<callable_t>::add_cb(callable_t cb, int severity, bool call_on_raise)
{
  add_cb(cb, severity, severity, call_on_raise);
}

template <typename callable_t>
void AlarmListener<callable_t>::add_clear_cb(callable_t cb)
{
  add_cb(cb, -1, -1, false);
}

template <typename callable_t>
void AlarmListener<callable_t>::__alarm_update(ros_alarms::Alarms alarms)
{
  // Check update list for our alarm name
  for(auto alarm_msg : alarms.alarms)
  {
    if(alarm_msg.alarm_name == __last_alarm.alarm_name)
    {
      // Update internal alarm data
      __last_alarm = alarm_msg;
      __last_update = ros::Time::now();
      
      // Invoke callbacks if necessary
      for(auto& cb : __callbacks)
        cb(alarm_msg);
    }
  }
}
