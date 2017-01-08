#pragma once

#include <ros/ros.h>

#include <string>

class AlarmListener
{
  AlarmListener(ros::NodeHandle &nh, std::string alarm_name);

};
