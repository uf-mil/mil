/**
 * Author: David Soto
 * Date: Jan 16, 2017
 */
#pragma once

#include <ros/ros.h>
#include <ros_alarms_msgs/Alarm.h>

#include <sstream>
#include <string>

namespace ros_alarms
{
/**
 * Representation of a ROS alarm. Used by several other classes to facilitate proper
 * interactions between components.
 */
struct AlarmProxy
{
  /**
   * The default class constructor. All parameters are empty or zero.
   */
  AlarmProxy()
  {
  }

  /**
   * A constructor which has parameters for all class parameters.
   */
  AlarmProxy(std::string alarm_name,  // full ctor
             bool raised, std::string node_name, std::string problem_description, std::string json_parameters,
             int severity);

  /**
   * A constructor with parameters for all class attributes, except the node name,
   * which is derived from the current node name (using ``ros::this_node``).
   */
  AlarmProxy(std::string alarm_name,  // ctor without node name
             bool raised, std::string problem_description, std::string json_parameters, int severity);

  /**
   * Constructs the class from a ROS message.
   *
   * @param msg The message to construct the class from.
   */
  AlarmProxy(ros_alarms_msgs::Alarm msg);

  /**
   * Converts the Alarm Proxy object to an :class:`Alarm` message.
   *
   * @return The constructed message.
   */
  ros_alarms_msgs::Alarm as_msg();  // convert to ros msg

  /**
   * Prints a readable representation of the class. Normally prints the alarm name,
   * alarm status, and whether the alarm is raised or cleared. If raised, the severity
   * is also printed.
   *
   * The full representation also prints the node name, a problem description,
   * and any related JSON parameters.
   *
   * @param full Whether to print the full description.
   */
  std::string str(bool full = false) const;  // Returns printable representation of AlarmProxy

  /**
   * Overloaded equality operator. Returns true if both alarms share the same name,
   * raised status, node name, problem description, and status.
   *
   * @param other The other alarm.
   */
  bool operator==(const AlarmProxy &other) const;

  // Public fields
  /**
   * The alarm name.
   */
  std::string alarm_name;

  /**
   * Whether the alarm has been raised by some service.
   */
  bool raised = false;

  /**
   * The name of the node connected to the alarm.
   */
  std::string node_name;

  /**
   * The description of the problem associated with the alarm.
   */
  std::string problem_description;

  /**
   * The JSON parameters associated with the alarm.
   */
  std::string json_parameters;
  int severity = 0;
};

}  // namespace ros_alarms
