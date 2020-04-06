#pragma once

#include <indyav_gazebo/wheel_plugin.hpp>
#include <indyav_control/ThrustStamped.h>

namespace gazebo
{
class BackWheelPlugin : public WheelPlugin<indyav_control::ThrustStamped>
{
public:
  BackWheelPlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();
  virtual void Callback(const indyav_control::ThrustStamped& _msg);
  virtual void OnUpdate();

protected:
  physics::LinkPtr base_link_;

  ignition::math::Vector3<double> thrust_ = ignition::math::Vector3<double>::Zero;

  double max_thrust_ = 0.0;
  double max_velocity_ = 0.0;

};

GZ_REGISTER_MODEL_PLUGIN(BackWheelPlugin)
}
