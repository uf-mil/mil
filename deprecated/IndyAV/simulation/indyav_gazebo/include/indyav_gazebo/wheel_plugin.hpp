#pragma once

#include <ros/ros.h>
#include <gazebo/common/Event.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
template <class MSG>
class WheelPlugin : public ModelPlugin
{
public:
  WheelPlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init() = 0;
  virtual void Callback(const MSG& _msg) = 0;
  virtual void OnUpdate() = 0;

protected:
  std::vector<std::string> wheel_names_;

  physics::ModelPtr model_;

  ros::NodeHandle nh_;

  ros::Subscriber sub_;

  event::ConnectionPtr updateConnection_;
};
}
#include "../src/wheel_plugin.cpp"
