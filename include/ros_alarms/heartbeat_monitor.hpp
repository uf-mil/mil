/**
 * Author: David Soto
 * Date: Jan 16
 */
#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Header.h>

#include <ros_alarms/broadcaster.hpp>

#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <cstdio>

namespace ros_alarms
{

template <typename msg_t = std_msgs::Header>
class HeartbeatMonitor
{
public:
  HeartbeatMonitor(ros::NodeHandle nh,
                   std::string alarm_name,
                   std::string heartbeat_topic,
                   std::function<bool(msg_t)> predicate = [](msg_t){ return true;},
                   ros::Duration time_to_raise = ros::Duration(0.1),
                   ros::Duration time_to_clear = ros::Duration(0.1),
                   ros::Duration period = ros::Duration(0.02));
                   // TODO: find out why a period < 0.02 causes callbacks not to be called
  std::string alarm_name()     const { return __alarm_proxy.alarm_name; }
  std::string heartbeat_name() const { return __heartbeat_topic; }
  bool healthy()               const { return __healthy; }
  ros::Time getLastBeatTime()  const { return __last_beat; }
  int getNumConnections()      const { return __heartbeat_listener.getNumPublishers(); }
  bool waitForConnection(ros::Duration timeout = {-1.0}) const;  // waits forever by default
  void startMonitoring();
  void stopMonitoring() { __async_spinner.stop(); }

private:
  ros::NodeHandle __nh;
  std::string __heartbeat_topic;
  AlarmProxy __alarm_proxy;
  std::function<bool(msg_t)> __msg_predicate;
  AlarmBroadcaster __alarm_broadcaster;
  AlarmListener<> __alarm_listener;
  ros::CallbackQueue __cb_queue;
  ros::AsyncSpinner __async_spinner;
  ros::Subscriber __heartbeat_listener;
  ros::Timer __status_checker;
  ros::Duration __time_to_raise;
  ros::Duration __time_to_clear;
  ros::Duration __period;
  ros::Time __last_beat = {0, 0};
  ros::Duration __time_recovering = {0, 0};
  bool __recovering = false;
  bool __healthy = true;
  std::string __object_name;
  void __heardHeartbeat(msg_t beat_msg);  // Callback for heartbeat topic subscriber
  void __checkForHeartbeatLoss(const ros::TimerEvent &te);  // Callback for __status_checker
};

template <typename msg_t>
HeartbeatMonitor<msg_t>
::HeartbeatMonitor(ros::NodeHandle nh, std::string alarm_name, std::string heartbeat_topic,
                   std::function<bool(msg_t)> predicate, ros::Duration time_to_raise,
                   ros::Duration time_to_clear, ros::Duration period)
: __nh(nh),
  __heartbeat_topic(heartbeat_topic),
  __alarm_proxy(alarm_name, false, "", "", 5),
  __msg_predicate(predicate),
  __alarm_broadcaster(nh, &__alarm_proxy),  // Don't use the .alarm() method on this!
  __alarm_listener(nh, __heartbeat_topic),
  __async_spinner(1, &__cb_queue),
  __last_beat(ros::Time::now()),
  __time_to_raise(time_to_raise),
  __time_to_clear(time_to_clear),
  __period(period)
{
  // Things that depend on the privately owned CallbackQueue are initialized here
  //   because setCallbackQueue() has to be called firs
  __nh.setCallbackQueue(&__cb_queue);
  __heartbeat_listener = __nh.subscribe(__heartbeat_topic, 10,
                                        &HeartbeatMonitor::__heardHeartbeat, this);
  __status_checker = __nh.createTimer(__period, 
                                      &HeartbeatMonitor::__checkForHeartbeatLoss, this);

  // Register cleared alarm w/ alarm server
  __alarm_broadcaster.clear();

  // For better error msgs
  std::stringstream obj;
  obj << "HeartbeatMonitor[heartbeat_name=" << heartbeat_topic
      << ", alarm_name=" << __alarm_proxy.alarm_name
      << ", node=" << __alarm_proxy.node_name << "]";
  __object_name = obj.str();

  std::stringstream init_msg;
  init_msg << "Node " << __alarm_proxy.node_name << " is now monitoring the following"
    " heartbeat: " << __heartbeat_topic;
  //ROS_INFO("%s", init_msg.str().c_str());
}

template <typename msg_t>
void HeartbeatMonitor<msg_t>
::__heardHeartbeat(msg_t beat_msg)
{
  auto receipt_time = ros::Time::now();
  auto last_beat_time  = __last_beat;
  bool valid_beat = false;

  // Call predicate and set last healthy beat time
  try
  {
     valid_beat = __msg_predicate(beat_msg);
  }
  catch(std::exception &e)
  {
     std::stringstream err;
     err << __object_name << " - Predicate function threw an exception: " << e.what();
     ROS_WARN("%s", err.str().c_str());
  }

  if(!valid_beat) // Don't do stuff if beat is invalid
    return;

  __last_beat = receipt_time; // update __last_beat
  auto time_since_prev_beat = receipt_time - last_beat_time;

  // Set the last healthy beat time as a json param on the alarm proxy
  std::stringstream json;
  // not exactly sure if this will work with json.loads()
  json << "{last_heathy_beat : " <<  __last_beat.toSec() << "}";
  __alarm_proxy.json_parameters = json.str();

  if(__healthy)  // We're done if the heartbeat is healthy
    return;

  // Now we will determine if we need to start recovering or increase the
  //   recovery counter or if we are fully recovered and need to clear alarm

  if(__recovering)  // Either increase recovery time or reset it
  {
    if(time_since_prev_beat < __time_to_raise)
    {
      __time_recovering += time_since_prev_beat;
      if(__time_recovering > __time_to_raise)
      {
        // Reset health flag and exit recovery mode
        __healthy = true;
        __recovering = false;

        // Clear heartbeat-loss alarm
        __alarm_broadcaster.clear();
      }
    }
    else
    {
      __time_recovering = ros::Duration(0);
    }
  }
  else  // Set flag for recovery mode
  {
    __recovering = true;
    __time_recovering = ros::Duration(0.0);
  }
}

template <typename msg_t>
void HeartbeatMonitor<msg_t>
::__checkForHeartbeatLoss(const ros::TimerEvent &te)
{
  if(!__healthy) // Can't lose what's already lost
    return;

  ros::Duration time_since_last_beat = ros::Time::now() - __last_beat;

  if(time_since_last_beat > __time_to_raise) // Raise heartbeat-loss alarm
  {
    __alarm_proxy.raised = true;
    __alarm_broadcaster.raise();
    __healthy = false;
  }
  // Should we add a mode where we increase the severity based on the time the
  //   heartbeat has been lost?
}

template <typename msg_t>
void HeartbeatMonitor<msg_t>
::startMonitoring()
{
  __healthy = __alarm_listener.queryCleared();
  __recovering = false;
  __last_beat = ros::Time::now(); // Comparisons for time_to_raised or time_to_clear
  __cb_queue.clear();             //   will be based off this time
  __async_spinner.start();
}


template <typename msg_t>
bool HeartbeatMonitor<msg_t>
::waitForConnection(ros::Duration timeout) const
{
  auto period = ros::Duration(0.01);
  auto start = ros::Time::now();
  while(timeout > ros::Time::now() - start)
  {
    if(getNumConnections() > 0)
      return true;
    else
      period.sleep();
  }
  return false;
}

}  // namespace ros_alarms
