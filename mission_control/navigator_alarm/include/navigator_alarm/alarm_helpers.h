/**
 * Author: Patrick Emami
 * Date: 11/9/2015
 */

#ifndef _ALARM_HELPERS_H
#define _ALARM_HELPERS_H

#include <ros/ros.h>
#include <string>
#include <boost/filesystem.hpp>  // directory iterators, etc
#include <map>
#include "navigator_msgs/Alarm.h"

namespace fs = ::boost::filesystem;

namespace navigator {

class AlarmRaiser;
typedef boost::shared_ptr<AlarmRaiser> AlarmRaiserPtr;

class AlarmBroadcaster;
typedef boost::shared_ptr<AlarmBroadcaster> AlarmBroadcasterPtr;

typedef boost::shared_ptr<ros::Publisher> PublisherPtr;

class AlarmBroadcaster {
 public:
  // Initiates a ros publisher for sending alarms on the /alarm topic
  AlarmBroadcaster(boost::shared_ptr<ros::NodeHandle> n);

  // Searches for an alarm JSON file with the given alarm name,
  // parses, and auto-generates it.
  // Returns the new alarm, or a nullptr on failure
  AlarmRaiserPtr addJSONAlarm(const std::string& name);

  // Factory method for creating individual AlarmRaiser objects
  AlarmRaiserPtr addAlarm(const std::string& name, bool action_required = true,
                          int severity = 2,
                          const std::string& problem_description = "",
                          const std::string& parameters = "");

 private:
  fs::path _dirname;
  const std::string _node_name = ros::this_node::getName();
  std::map<std::string, AlarmRaiserPtr> _alarms;
  PublisherPtr _alarm_publisher;
};

class AlarmRaiser {
 public:
  // Used to generate alarm messages
  AlarmRaiser(const std::string& alarm_name, const std::string& node_name,
              PublisherPtr& alarm_publisher, bool action_required = true,
              int severity = 2, const std::string& problem_description = "",
              const std::string& parameters = "");

  boost::shared_ptr<navigator_msgs::Alarm> raiseAlarm(
      const std::string& problem_description = "",
      const std::string& parameters = "") const;

  // Getters for alarm info
  const std::string getAlarmName() const;
  const std::string getNodeName() const;
  const std::string getProblemDescription() const;

  bool isActionRequired() const;
  int getSeverity() const;

 private:
  const std::string _alarm_name;
  const std::string _node_name;
  const std::string _problem_description;
  const std::string _parameters;

  bool _action_required;
  int _severity;

  PublisherPtr _alarm_publisher;
};
}

#endif /* ALARM_HELPERS_H_ */
