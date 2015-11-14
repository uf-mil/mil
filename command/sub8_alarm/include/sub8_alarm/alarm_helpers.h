/**
 * Author: Patrick Emami
 * Date: 11/9/2015
 */

#include <ros/ros.h>
#include <string>
#include <cstddef>

#include "sub8_msgs/Alarm.h"

namespace sub8 {

class AlarmRaiser;
typedef boost::shared_ptr<AlarmRaiser> AlarmRaiserPtr;

class AlarmBroadcaster;
typedef boost::shared_ptr<AlarmBroadcaster> AlarmBroadcasterPtr;

typedef boost::shared_ptr<ros::Publisher> PublisherPtr; 

class AlarmBroadcaster {
 public:
  // Initiates a ros publisher for sending alarms on the /alarm topic
  AlarmBroadcaster(boost::shared_ptr<ros::NodeHandle> n);

  // Factory method for creating new AlarmRaiser objects
  AlarmRaiserPtr addAlarm(const std::string& name, bool action_required = false,
                          int severity = 2,
                          const std::string& problem_description = "",
                          const std::string& parameters = "");

 private:
  const std::string _node_name = ros::this_node::getName();
  std::vector<AlarmRaiserPtr> _alarms;
  PublisherPtr _alarm_publisher;
};

class AlarmRaiser {
 public:
  // Used to generate alarm messages
  AlarmRaiser(const std::string& alarm_name, const std::string& node_name,
              PublisherPtr& alarm_publisher, bool action_required = false,
              int severity = 2, const std::string& problem_description = "",
              const std::string& parameters = "");

  boost::shared_ptr<sub8_msgs::Alarm> raiseAlarm(
      const std::string& problem_description, const std::string& parameters) const;

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