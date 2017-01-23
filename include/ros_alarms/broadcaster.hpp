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
  AlarmBroadcaster(ros::NodeHandle &nh, AlarmProxy* alarm = nullptr);
  void clear() { __alarm_ptr->raised = false; publish(); }
  void raise() { __alarm_ptr->raised = true;  publish(); }
  void updateSeverity(uint8_t sev) { __alarm_ptr->severity = sev; publish();}
  AlarmProxy& alarm() { return *__alarm_ptr; }

private:
  ros::NodeHandle __nh;
  AlarmProxy* __alarm_ptr;  // All code should refer to the proxy via ptr
  ros::ServiceClient __set_alarm;
  AlarmProxy __alarm_proxy; // This allows internal management of the alarm proxy
  bool publish();
};

}  // namespace ros_alarms
