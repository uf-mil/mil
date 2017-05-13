/**
 * Author: David Soto
 * Date: Jan 16, 2017
 */
#pragma once

#include <ros/ros.h>
#include <ros_alarms/Alarm.h>

#include <sstream>
#include <string>

namespace ros_alarms
{
struct AlarmProxy
{
  AlarmProxy()
  {
  }  // Default ctor (cleared, severity = 0, empty strings

  AlarmProxy(std::string alarm_name,  // full ctor
             bool raised, std::string node_name, std::string problem_description, std::string json_parameters,
             int severity);

  AlarmProxy(std::string alarm_name,  // ctor without node name
             bool raised, std::string problem_description, std::string json_parameters, int severity);

  AlarmProxy(ros_alarms::Alarm msg);  // ctor from ros msg

  Alarm as_msg();  // convert to ros msg

  std::string str(bool full = false) const;  // Returns printable representaion of AlarmProxy

  bool operator==(const AlarmProxy &other) const;

  // Public fields
  std::string alarm_name;
  bool raised = false;
  std::string node_name;
  std::string problem_description;
  std::string json_parameters;
  int severity = 0;
};

}  // namespace ros_alarms
