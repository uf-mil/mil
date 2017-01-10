#pragma once

#include <ros/ros.h>
#include <ros_alarms/AlarmSet.h>
#include <ros_alarms/Alarm.h>

#include <alarm_proxy.hpp>

#include <string>


class AlarmBroadcaster
{
public:
  AlarmBroadcaster(ros::NodeHandle &nh, AlarmProxy* alarm);
  void clear() { __alarm_ptr->raised = false; publish(); }
  void raise() { __alarm_ptr->raised = true;  publish(); }
  void updateSeverity(uint8_t sev) { __alarm_ptr->severity = sev; publish();}
  AlarmProxy& alarm() { return *__alarm_ptr; }

private:
  ros::NodeHandle __nh;
  ros::ServiceClient __set_alarm;
  AlarmProxy* __alarm_ptr;
  bool publish();
};


