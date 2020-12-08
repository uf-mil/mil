#pragma once

#include <indyav_control/ThrottleBrakeStamped.h>
#include <indyav_gazebo/wheel_plugin.hpp>

namespace gazebo
{
class BackWheelPlugin : public WheelPlugin<indyav_control::ThrottleBrakeStamped>
{
public:
  BackWheelPlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();
  virtual void Callback(const indyav_control::ThrottleBrakeStamped& _msg);
  virtual void OnUpdate();

protected:
  std::string back_axle_joint_name_;
  physics::JointPtr back_axle_joint_;

  double cmd_ = 0.0;
  double engine_to_wheel_ratio_ = 0.0;
  double wheel_radius_ = 0.0;
  double max_velocity_ = 0.0;
};

GZ_REGISTER_MODEL_PLUGIN(BackWheelPlugin)
}
