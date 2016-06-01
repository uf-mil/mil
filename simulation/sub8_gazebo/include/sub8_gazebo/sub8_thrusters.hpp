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
#include "sub8_msgs/Thrust.h"
#include "sub8_msgs/ThrusterCmd.h"
#include <ros/ros.h>

namespace gazebo {

struct Thruster {
  Thruster() {
    min = 0;
    max = 0;
  }
  Thruster(std::string param_name) {
    std::vector<double> v_direction, v_position, v_bounds;
    ros::param::get(param_name + "/direction", v_direction);
    ros::param::get(param_name + "/position", v_position);
    ros::param::get(param_name + "/bounds", v_bounds);
    position = math::Vector3(v_position[0], v_position[1], v_position[2]);
    direction = math::Vector3(v_direction[0], v_direction[1], v_direction[2]);
    min = v_bounds[0];
    max = v_bounds[1];
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

  // Store
  // math::Vector3 netForce;
  // math::Vector3 netTorque;

  // protected: double fluidDensity;
  // protected: double dragCoeff;
  // protected: std::map<int, VolumeProperties> volPropsMap;
};
}
