#pragma once

#include <ros/ros.h>

#include <string>


class AlarmBroadcaster
{
  AlarmBroadcaster(ros::NodeHandle &nh, std::string alarm_name);
  ros::ServiceClient __set_alarm;

};


