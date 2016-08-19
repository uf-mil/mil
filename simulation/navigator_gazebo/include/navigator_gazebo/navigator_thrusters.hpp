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
#include "roboteq_msgs/Command.h"
#include <ros/ros.h>

namespace gazebo {


struct Thruster {
  Thruster(){ }
  Thruster(std::string param_name) {
    std::vector<double> v_position;
    double v_effort_limit, v_effort_ratio, v_rotation;

    ros::param::get("/thrust_mapper/thruster_" + param_name + "_cog", v_position);
    ros::param::get("/thrust_mapper/thruster_" + param_name + "_theta", v_rotation);
    ros::param::get("/thrust_mapper/effort_ratio", v_effort_ratio);
    ros::param::get("/thrust_mapper/effort_limit", v_effort_limit);

    position = math::Vector3(v_position[0], v_position[1], 0);

    // Direction is give as rotation about the z axis.
    // This assumes that it's ONLY a 2d rotation about the z-axis.
    direction = math::Vector3(cos(v_rotation), sin(v_rotation), 0);

    ROS_INFO("%s position: %f, %f, %f", param_name.c_str(), position[0], position[1], position[2]);
    ROS_INFO("%s direction: %f, %f, %f", param_name.c_str(), direction[0], direction[1], direction[2]);

    effort_ratio = v_effort_ratio;
    effort_limit = v_effort_limit;
  }
  math::Vector3 position, direction;
  double effort_ratio, effort_limit;
};

class ThrusterPlugin : public ModelPlugin {
 public:
  ThrusterPlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();
  void BLCallback(const roboteq_msgs::Command::ConstPtr &command);
  void BRCallback(const roboteq_msgs::Command::ConstPtr &command);
  void FLCallback(const roboteq_msgs::Command::ConstPtr &command);
  void FRCallback(const roboteq_msgs::Command::ConstPtr &command);

 protected:
  ros::NodeHandle nh;
  ros::Subscriber blSub;
  ros::Subscriber brSub;
  ros::Subscriber flSub;
  ros::Subscriber frSub;
  virtual void OnUpdate();

 protected:
  std::mutex mtx;
  event::ConnectionPtr updateConnection;
  physics::ModelPtr model;
  physics::PhysicsEnginePtr physicsEngine;
  sdf::ElementPtr sdf;
  std::string layoutParam;
  physics::LinkPtr targetLink;
  ros::Time lastTime;

  std::vector<double> commands {0, 0, 0, 0};
  std::vector<Thruster> thrusters;  // BL, BR, FL, FR
};
}
