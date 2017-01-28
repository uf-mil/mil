/**
 * Author: David Soto
 * Date: Jan 16
 */
#pragma once

#include <ros/ros.h>
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
  HeartbeatMonitor(ros::NodeHandle nh, std::string alarm_name, std::string heartbeat_topic,
                   std::function<bool(msg_t)> predicate, ros::Duration time_to_raise= {0.1, 0},
                   ros::Duration time_to_clear={0.1, 0}, ros::Duration init_period = {0.1});
  std::string alarm_name() const { return __alarm_proxy.alarm_name; }
  std::string heartbeat_name() const { return __heartbeat_topic; }
  bool healthy() const { return __healthy; }
  ros::Time last_beat() { return __last_beat; }
  int getNumConnections() { return __heartbeat_listener.getNumPublishers(); }

private:
  ros::NodeHandle __nh;
  std::string __heartbeat_topic;
  AlarmProxy __alarm_proxy;
  std::function<bool(msg_t)> __msg_predicate;
  AlarmBroadcaster __alarm_broadcaster;
  ros::Subscriber __heartbeat_listener;
  ros::Timer __status_checker;
  ros::Duration __time_to_raise;
  ros::Duration __time_to_clear;
  ros::Duration __init_period;
  ros::Time __init_time;
  ros::Time __last_beat = {0, 0};
  ros::Duration __time_recovering = {0, 0};
  bool __recovering = false;
  bool __healthy = true;
  void __record_heartbeat(msg_t beat_msg);  // Callback for heartbeat topic subscriber
  void __diagnose_heartbeat(const ros::TimerEvent &te);  // Callback for __status_checker
};

template <typename msg_t>
HeartbeatMonitor<msg_t>
::HeartbeatMonitor(ros::NodeHandle nh, std::string alarm_name, std::string heartbeat_topic,
                   std::function<bool(msg_t)> predicate, ros::Duration time_to_raise,
                   ros::Duration time_to_clear, ros::Duration init_period)
: __nh(nh),
  __heartbeat_topic(heartbeat_topic),
  __alarm_proxy(alarm_name, false, "", "", 5),
  __alarm_broadcaster(nh, &__alarm_proxy),  // Don't use the .alarm() method on this!
  __heartbeat_listener(__nh.subscribe(__heartbeat_topic, 10, 
                                      &HeartbeatMonitor::__record_heartbeat, this)),
  __status_checker(
    __nh.createTimer(ros::Duration(std::min(time_to_raise, time_to_clear).toSec() / 5.0),
                     &HeartbeatMonitor::__diagnose_heartbeat, this)),
  __time_to_raise(time_to_raise),
  __time_to_clear(time_to_clear),
  __init_period(init_period),
  __init_time(ros::Time::now())
{
  std::stringstream init_msg;
  init_msg << "Node " << __alarm_proxy.node_name << " is now monitoring the following"
    << " Heartbeat topic: " << __heartbeat_topic;
  std::cerr << init_msg.str() << std::endl;
  ROS_INFO("%s", init_msg.str().c_str());
}

template <typename msg_t>
void HeartbeatMonitor<msg_t>
::__record_heartbeat(msg_t beat_msg)
{
  std::cerr << "Got Heartbeat" << std::endl;
  // Call predicate and set last healthy beat time
  auto receipt_time = ros::Time::now();
  bool valid_beat = false;

  try
  {
     valid_beat = __msg_predicate(beat_msg);
  }
  catch(std::exception &e)
  {
     std::stringstream err;
     err << __alarm_proxy.alarm_name << " HeartbeatMonitor - Predicate function threw an "
       "exeption: " << e.what();
     ROS_WARN("%s", err.str().c_str());
  }

  if(valid_beat)
  {
    __last_beat = receipt_time;

    // Set the last healthy beat time as a json param on the alarm proxy
    std::stringstream json;
    // not exactly sure if this will work with json.loads()
    json << "{last_heathy_beat : " <<  __last_beat.toSec() << "}";
    __alarm_proxy.json_parameters = json.str();

   // We're done if the heartbeat is healthy
   if(__healthy)
     return;

    // If the heartbeat was not healthy, check if it was recovering
    if(__recovering)  // Increase the recovery timer
      __time_recovering += ros::Time::now() - __last_beat;
    else  // Set flag for recovery mode
    {
      __recovering = true;
      __time_recovering = ros::Duration(0.0);
    }
  }
}

template <typename msg_t>
void HeartbeatMonitor<msg_t>
::__diagnose_heartbeat(const ros::TimerEvent &te)
{
  std::cerr << "Checking heartbeat status, " << ros::Time::now() << std::endl;
  // Case 1: Within Init Period
  //   won't raise alarm even if healthy heartbeat not detected
  if(ros::Time::now() - __init_time < __init_period)
    return;

  // Case 2: Heartbeat was healthy
  if(__healthy)
  {
    ros::Duration time_since_last_beat = ros::Time::now() - __last_beat;
  
    if(time_since_last_beat > __time_to_raise) // Raise heartbeat-loss alarm
    {
      __alarm_proxy.raised = true;
      __alarm_broadcaster.raise();
      __healthy = false;
      std::cerr << "Raising lost heartbeat alarm" << ros::Time::now() << std::endl;
    }
    // Should we add a mode where we increase the severity based on the time the
    //   heartbeat has been lost?
  }

  // Case 3: Heartbeat was not healthy
  else
  {
    // Heartbeat has finished recovering
    if(__recovering && __time_recovering > __time_to_clear)
    {
      // Reset health flag and exit recovery mode
      __healthy = true;
      __recovering = false;

      // Clear heartbeat-loss alarm
      __alarm_proxy.raised = true;
      __alarm_broadcaster.clear();
    }
    else {}  // Either not fully recovered or not recovering at all
  }
}


}  // namespace ros_alarms
