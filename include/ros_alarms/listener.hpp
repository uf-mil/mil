#pragma once

#include <ros/ros.h>
#include <ros_alarms/Alarm.h>
#include <ros_alarms/AlarmGet.h>

#include <ros_alarms/alarm_proxy.hpp>

#include <string>
#include <functional>

namespace ros_alarms
{

template <typename callable_t = std::function< void(AlarmProxy) >>
struct ListenerCb
{
  enum class CallScenario { raise, clear, always };

  callable_t  cb_func;  // object needs to have a call operator
  int severity_hi = 5;  // highest priority
  int severity_lo = 0;  // lowest priority
  CallScenario call_scenario = CallScenario::always;

  // Compares alarm severity against the action_required range for this callback
  bool severity_check(int severity)
    { return severity <= severity_hi && severity >= severity_lo; }

  // Checks msg to see if its raised status and severity require calling
  // the associated callbacks and calls them if necessary
  void operator()(ros_alarms::Alarm msg)
  {
    // Only call if alarm status matches the call_scenario
    bool call_scenario_match = call_scenario == CallScenario::always
      || (call_scenario == CallScenario::raise && msg.raised)
      || (call_scenario == CallScenario::clear && !msg.raised);
    bool needs_call = call_scenario_match &&
      (call_scenario == CallScenario::raise? severity_check(msg.severity) : true);

    if(needs_call)
    {
      AlarmProxy alarm { msg };
      cb_func(alarm);
    }    
  }
};

template <typename callable_t = std::function< void(AlarmProxy) >>
class AlarmListener
{
  using CallScenario = typename ListenerCb<callable_t>::CallScenario;

public:
  AlarmListener(ros::NodeHandle &nh, std::string alarm_name);

  // Return status of listener
  bool ok() const { return __ok; }

  // Functions that return the status of the alarm at time of last update
  bool is_raised() const        { return __last_alarm.raised; }
  bool is_cleared() const       { return !is_raised(); }

  // Functions that query the server before returning the latest status of the alarm
  bool query_raised()           { get_alarm(); return __last_alarm.raised; }
  bool query_cleared()          { get_alarm(); return !is_raised(); }

  // Queries server and parses response into an AlarmProxy
  AlarmProxy get_alarm();

  // Returns AlarmProxy for last alarm of this name published to '/alarm/updates'
  AlarmProxy get_cached_alarm() { return __last_alarm; }

  // Registers a callback to be invoked on both raise and clear
  void add_cb(callable_t cb);

  // Registers a raise callback to be called for any severity level
  void add_raise_cb(callable_t cb);

  // Registers a raise callback to be called for a single severity level
  void add_raise_cb(callable_t cb, int severity);

  // Registers a callback to be invoked at alarm raise for a range of severities
  void add_raise_cb(callable_t cb, int severity_lo, int severity_hi);

  // Registers a callback to be invoked when alarm is cleared
  void add_clear_cb(callable_t cb);

  // Returns the last time the alarm was updated
  ros::Time last_update_time() const { return __last_update; }

  // Returns the time since last update
  ros::Duration time_since_update() const { return ros::Time::now() - __last_update; }

  void clear_callbacks()      { __callbacks.clear(); }

private:
  bool __ok = true;  // listener fail-bit
  ros::NodeHandle __nh;
  std::string __alarm_name;
  ros::ServiceClient __get_alarm;
  ros::Subscriber __update_subscriber;
  std::vector<ListenerCb<callable_t>> __callbacks;
  AlarmProxy __last_alarm;
  ros::Time __last_update { 0, 0 };
  void __add_cb(callable_t cb, int severity_lo, int severity_hi, CallScenario call_scenario);
  void __alarm_update(ros_alarms::Alarm);
};


template <typename callable_t>
AlarmListener<callable_t>
::AlarmListener(ros::NodeHandle &nh, std::string alarm_name)
: __nh(nh), __alarm_name(alarm_name)
{
  // Service to query alarm server
  __get_alarm = __nh.serviceClient<ros_alarms::AlarmGet>("/alarm/get");
  bool service_exists = __get_alarm.waitForExistence(ros::Duration(2.0));
  if(!service_exists)
  {
    __ok = false;
    std::string msg = __PRETTY_FUNCTION__;
    msg += ": timed out waiting for service /alarm/get";
    ROS_WARN(msg.c_str());
  }

  try
  {
    // Subscribes to list of recently modified alarms from alarm server
    __update_subscriber = __nh.subscribe("/alarm/updates", 1000,
                                         &AlarmListener<callable_t>::__alarm_update, this);
    // Initial server query so stored AlarmProxy is initialized
    get_alarm();
  }
  catch(std::exception &e)
  {
    __ok = false;
    std::cerr << __PRETTY_FUNCTION__ << e.what() << std::endl;
    ROS_WARN((std::string(__PRETTY_FUNCTION__) + ": " + e.what()).c_str());
  }

  // Make sure that the update subscriber is connected to something or you wont get cb's
  ros::Duration wait{2.0};
  ros::Duration period{0.05};  // or else it's not interesting
  ros::Time start = ros::Time::now();
  while(__update_subscriber.getNumPublishers() <= 0)
  {
    if(ros::Time::now() - start > wait)
    {
      __ok = false;
      break;
    }
  }
}

template <typename callable_t>
AlarmProxy AlarmListener<callable_t>
::get_alarm()
{
  // Create Query msg
  ros_alarms::AlarmGet alarm_query;
  alarm_query.request.alarm_name = __alarm_name;

  // Query alarm server
  if(!__get_alarm.call(alarm_query))
    ROS_INFO("Alarm server query was unsuccessful.");

  // Update internal alarm data
  __last_update = alarm_query.response.header.stamp;
  __last_alarm = AlarmProxy(alarm_query.response.alarm);
  return __last_alarm;
}

template <typename callable_t>
void AlarmListener<callable_t>
::add_cb(callable_t cb)
{
  __add_cb(cb, 0, 5, CallScenario::always);
}

template <typename callable_t>
void AlarmListener<callable_t>
::add_raise_cb(callable_t cb)
{
  __add_cb(cb, 0, 5, CallScenario::raise);
}

template <typename callable_t>
void AlarmListener<callable_t>
::add_raise_cb(callable_t cb, int severity)
{
  __add_cb(cb, severity, severity, CallScenario::raise);
}

template <typename callable_t>
void AlarmListener<callable_t>
::add_raise_cb(callable_t cb, int severity_lo, int severity_hi)
{
  __add_cb(cb, severity_lo, severity_hi, CallScenario::raise);
}

template <typename callable_t>
void AlarmListener<callable_t>
::add_clear_cb(callable_t cb)
{
  __add_cb(cb, 0, 5, CallScenario::clear);
}

template <typename callable_t>
void AlarmListener<callable_t>
::__add_cb(callable_t cb, int severity_lo, int severity_hi,
           CallScenario call_scenario)
{
  ListenerCb<callable_t> l_cb;
  l_cb.cb_func = cb;
  l_cb.severity_lo = severity_lo;
  l_cb.severity_hi = severity_hi;
  l_cb.call_scenario = call_scenario;

  __callbacks.push_back(l_cb);
}

template <typename callable_t>
void AlarmListener<callable_t>
::__alarm_update(ros_alarms::Alarm alarm_msg)
{
  if(alarm_msg.alarm_name == __alarm_name)
  {
    // Update internal alarm data
    __last_alarm = alarm_msg;
    __last_update = ros::Time::now();
 
    // Invoke callbacks if necessary
    for(auto& cb : __callbacks)
      cb(alarm_msg);
  }
}

}  // namespace ros_alarms
