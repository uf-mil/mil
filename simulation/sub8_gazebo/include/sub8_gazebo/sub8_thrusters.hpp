#pragma once

#include <mutex>
#include <algorithm>
#include <vector>
#include <map>
#include <cmath>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/physics.hh"
#include <sub8_msgs/Thrust.h>
#include <sub8_msgs/ThrusterCmd.h>
#include <sub8_gazebo/sub8_thruster_config.hpp>
#include <ros/ros.h>

namespace gazebo {

struct Thruster {
  Thruster() {
    min = 0;
    max = 0;
  }
  Thruster(sub8_gazebo::ThrusterDef td) {
    position = math::Vector3(td.position[0], td.position[1], td.position[2]);
    direction = math::Vector3(td.direction[0], td.direction[1], td.direction[2]);
    min = td.bounds[0];
    max = td.bounds[1];
  }
  math::Vector3 position;
  math::Vector3 direction;
  double min;
  double max;
};

class ThrusterPlugin : public ModelPlugin {
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
