#pragma once

#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <vector>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/physics.hh"
#include "roboteq_msgs/Command.h"
#include "sensor_msgs/JointState.h"

namespace navigator_gazebo
{
class Thruster
{
public:
  Thruster(std::string _name, gazebo::physics::LinkPtr _link, gazebo::physics::JointPtr _joint);
  void Update(double& position, double& velocity, double& effort);

private:
  void CommandCallback(const roboteq_msgs::Command& _cmd);

  gazebo::physics::LinkPtr link_;
  gazebo::physics::JointPtr joint_;
  ros::NodeHandle nh_;
  ros::Subscriber command_sub_;
  std::mutex mutex_;
  double command_;
  ros::Time last_command_time_;
};

class ThrusterPlugin : public gazebo::ModelPlugin
{
public:
  ThrusterPlugin();
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const gazebo::common::UpdateInfo&);

private:
  ros::NodeHandle nh_;
  std::vector<std::unique_ptr<Thruster> > thrusters_;
  ros::Publisher joint_state_pub_;
  sensor_msgs::JointState joint_state_msg_;
  gazebo::event::ConnectionPtr update_connection_;
};
}
