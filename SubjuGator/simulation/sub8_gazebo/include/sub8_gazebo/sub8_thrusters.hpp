#pragma once

#include <ros/ros.h>
#include <sub8_msgs/Thrust.h>
#include <sub8_msgs/ThrusterCmd.h>
#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <sub8_gazebo/sub8_thruster_config.hpp>
#include <vector>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
struct Thruster
{
  Thruster()
  {
    min = 0;
    max = 0;
  }
  Thruster(sub8_gazebo::ThrusterDef td)
  {
    position = ignition::math::Vector3d(td.position[0], td.position[1], td.position[2]);
    direction = ignition::math::Vector3d(td.direction[0], td.direction[1], td.direction[2]);
    min = td.bounds[0];
    max = td.bounds[1];
  }
  ignition::math::Vector3d position;
  ignition::math::Vector3d direction;
  double min;
  double max;
};

class ThrusterPlugin : public ModelPlugin
{
public:
  ThrusterPlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();
  void ThrustCallback(const sub8_msgs::Thrust::ConstPtr &thrust);

protected:
  // virtual void OnUpdate();
  ros::NodeHandle nh;
  ros::Subscriber thrustSub;
  virtual void OnUpdate();

protected:
  std::mutex mtx;
  event::ConnectionPtr updateConnection;
  physics::ModelPtr model;
  physics::PhysicsEnginePtr physicsEngine;
  sdf::ElementPtr sdf;
  std::string layoutParam;
  double minAbsThrust;
  std::map<std::string, Thruster> thrusterMap;
  physics::LinkPtr targetLink;
  ros::Time lastTime;
  std::vector<sub8_msgs::ThrusterCmd> cmdQueue;
};
}
