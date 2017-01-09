#pragma once

#include <ros/ros.h>
#include <ros_alarms/AlarmGet.h>

#include <alarm_proxy.hpp>

#include <string>

class AlarmListener
{
  AlarmListener(ros::NodeHandle &nh, std::string alarm_name);

};
