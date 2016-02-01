/**
 * Author: Patrick Emami
 * Date: 11/9/2015
 */

#include "sub8_alarm/alarm_helpers.h"
#include "std_msgs/Header.h"
#include <ros/ros.h>
#include <string>
#include <boost/property_tree/ptree.hpp>        // ptree
#include <boost/property_tree/json_parser.hpp>  // read_json

using sub8::AlarmBroadcaster;
using sub8::AlarmRaiserPtr;
using sub8::AlarmRaiser;
using sub8::PublisherPtr;

AlarmBroadcaster::AlarmBroadcaster(boost::shared_ptr<ros::NodeHandle> n) {
  // create Publisher
  _alarm_publisher = boost::make_shared<ros::Publisher>(
      n->advertise<sub8_msgs::Alarm>("/alarm", 10));
}

AlarmRaiserPtr AlarmBroadcaster::addAlarm(
    const std::string& name, bool action_required, int severity,
    const std::string& problem_description, const std::string& parameters) {
  AlarmRaiserPtr new_alarm(new AlarmRaiser(name, _node_name, _alarm_publisher,
                                           action_required, severity,
                                           problem_description, parameters));
  // Add the new alarm to the list of alarms
  // maintained by the AlarmBroadcaster
  _alarms.insert(std::pair<std::string, AlarmRaiserPtr>(name, new_alarm));
  return new_alarm;
}

bool AlarmBroadcaster::addAlarms(const fs::path& dirname,
                                 std::map<std::string, AlarmRaiserPtr>& alarms) {
  std::vector<std::string> paths;
  const std::string ext = ".json";

  // alarm attributes
  std::string alarm_name;
  bool action_required;
  int severity;
  std::string problem_description;
  std::string parameters;

  if (!fs::exists(dirname) || !fs::is_directory(dirname)) {
    ROS_WARN("Could not find the following alarms directory %s",
             dirname.string().c_str());
    return false;
  }

  fs::recursive_directory_iterator it(dirname);
  fs::recursive_directory_iterator endit;

  // iterate over all files in the directory and store all with the extension in
  // a vector for further processing
  while (it != endit) {
    if (fs::is_regular_file(*it) && it->path().extension() == ext) {
      // Store the dirname + filename
      paths.push_back(it->path().string());
      ++it;
    }
  }

  // no json files were found
  if (paths.empty()) {
    ROS_WARN("Could not find any files with extension %s", ext.c_str());
    return false;
  }

  for (std::vector<std::string>::iterator it = paths.begin(); it != paths.end();
       ++it) {
    boost::property_tree::ptree pt;
    try {
      boost::property_tree::read_json(*it, pt);

      alarm_name = pt.get<std::string>("alarm_name");
      action_required = pt.get<bool>("action_required");
      severity = pt.get<int>("severity");
      problem_description = pt.get<std::string>("problem_description");
      parameters = pt.get<std::string>("parameters");
    } catch (const boost::property_tree::ptree_error& e) {
      ROS_WARN("%s", e.what());
      return false;
    }

    // add the new alarm
    addAlarm(alarm_name, action_required, severity, problem_description,
             parameters);
  }

  alarms = _alarms;

  return true;
}

AlarmRaiser::AlarmRaiser(const std::string& alarm_name,
                         const std::string& node_name,
                         PublisherPtr& alarm_publisher, bool action_required,
                         int severity, const std::string& problem_description,
                         const std::string& parameters)
    : _alarm_name(alarm_name),
      _node_name(node_name),
      _alarm_publisher(alarm_publisher),

      _action_required(action_required),
      _severity(severity),
      _problem_description(problem_description),
      _parameters(parameters) {
  // Verify severity is valid
  // Q: What do if it isn't?
  // A: Build an AI that will read over
  // our code and correct the severity levels
  // for us
}

boost::shared_ptr<sub8_msgs::Alarm> AlarmRaiser::raiseAlarm(
    const std::string& problem_description,
    const std::string& parameters) const {

  std::string pd =
      (problem_description != "") ? problem_description : _problem_description;

  std::string params = (parameters != "") ? parameters : _parameters;

  std_msgs::Header alarm_header;
  alarm_header.stamp = ros::Time::now();
  boost::shared_ptr<sub8_msgs::Alarm> alarm_msg(new sub8_msgs::Alarm());
  alarm_msg->header = alarm_header;
  alarm_msg->action_required = _action_required;
  alarm_msg->problem_description = problem_description;
  alarm_msg->parameters = params;
  alarm_msg->severity = _severity;
  alarm_msg->alarm_name = _alarm_name;
  alarm_msg->node_name = _node_name;

  _alarm_publisher->publish(alarm_msg);
  return alarm_msg;
}

const std::string AlarmRaiser::getAlarmName() const { return _alarm_name; }

const std::string AlarmRaiser::getNodeName() const { return _node_name; }

const std::string AlarmRaiser::getProblemDescription() const {
  return _problem_description;
}

bool AlarmRaiser::isActionRequired() const { return _action_required; }

int AlarmRaiser::getSeverity() const { return _severity; }