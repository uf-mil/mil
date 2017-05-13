/**
 * Author: David Soto
 * Date: Jan 16, 2017
 */
#include <ros_alarms/alarm_proxy.hpp>

using namespace std;

namespace ros_alarms
{
AlarmProxy::AlarmProxy(string _alarm_name, bool _raised, string _node_name, string _problem, string _json,
                       int _severity)
{
  alarm_name = _alarm_name;
  raised = _raised;
  node_name = _node_name;
  problem_description = _problem;
  json_parameters = _json;
  severity = _severity;
}

AlarmProxy::AlarmProxy(string _alarm_name, bool _raised, string _problem, string _json, int _severity)
{
  *this = AlarmProxy(_alarm_name, _raised, ros::this_node::getName(), _problem, _json, _severity);
}

AlarmProxy::AlarmProxy(ros_alarms::Alarm msg)
{
  alarm_name = msg.alarm_name;
  raised = msg.raised;
  node_name = msg.node_name;
  problem_description = msg.problem_description;
  json_parameters = msg.parameters;
  severity = msg.severity;
}

Alarm AlarmProxy::as_msg()
{
  ros_alarms::Alarm a;
  a.alarm_name = alarm_name;
  a.raised = raised;
  a.node_name = node_name;
  a.problem_description = problem_description;
  a.parameters = json_parameters;
  a.severity = severity;
  return a;
}

std::string AlarmProxy::str(bool full) const
{
  std::stringstream repr;
  repr << "[alarm_name: " << alarm_name << ", status: " << (raised ? "raised" : "cleared");
  if (raised)
    repr << ", severity: " << severity;
  if (full)
  {
    repr << ", node_name: " << (node_name.empty() ? "N/A" : node_name)
         << ", problem: " << (problem_description.empty() ? "N/A" : problem_description)
         << ", json_parameters: " << (json_parameters.empty() ? "N/A" : json_parameters);
  }
  repr << "]";
  return repr.str();
}

bool AlarmProxy::operator==(const AlarmProxy &other) const
{
  if (alarm_name != other.alarm_name)
    return false;
  if (raised != other.raised)
    return false;
  if (node_name != other.node_name)
    return false;
  if (problem_description != other.problem_description)
    return false;
  // JSON parameters can be stripped off by the alarm server if they can't be
  //   deserialized if(json_parameters != other.json_parameters) return false;
  if (severity != other.severity)
    return false;
  return true;
}

}  // namespace ros_alarms
