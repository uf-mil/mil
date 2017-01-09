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
  AlarmProxy& clear() { alarm->raised = false; publish(); }
  AlarmProxy& raise() { alarm->raised = true;  publish(); }
  AlarmProxy& updateSeverity(uint8_t sev) { alarm->severity = sev; publish();}
  AlarmProxy* alarm_ptr() { return alarm; }

private:
  ros::NodeHandle nh;
  ros::ServiceClient __set_alarm;
  AlarmProxy* alarm;
  bool publish();
};


