#pragma once

#include <ros/ros.h>
#include <ros_alarms/AlarmSet.h>
#include <ros_alarms/Alarm.h>

#include <ros_alarms/alarm_proxy.hpp>

#include <string>

namespace ros_alarms
{

class AlarmBroadcaster
{
public:
  // Constuct a broadcaster w/ an internally controlled (default) or externally controlled
  // (optional ptr argument) AlarmProxy
  AlarmBroadcaster(ros::NodeHandle &nh, AlarmProxy* alarm = nullptr);

  // Change alarm status and publish to server
  void clear() { __alarm_ptr->raised = false; publish(); }
  void raise() { __alarm_ptr->raised = true;  publish(); }

  // updateSeverity always raises the alarm because it doesn't make sense for
  // cleared alarm to have severity
  void updateSeverity(int sev) { __alarm_ptr->severity = sev; raise();}

  // Publishes current state of the AlarmProxy to the server
  bool publish();

  // Handle to update the AlarmProxy (if this is an externally managed AlPxy, then modifying
  // could potentially have side effects for other broadcasters that may use it. Exert Caution!)
  AlarmProxy& alarm() { return *__alarm_ptr; }

private:
  ros::NodeHandle __nh;
  AlarmProxy* __alarm_ptr;  // All code should refer to the proxy via ptr
  AlarmProxy __alarm_proxy; // This allows internal management of the alarm proxy
  ros::ServiceClient __set_alarm;
};

}  // namespace ros_alarms
