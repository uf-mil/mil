#pragma once

#include <indyav_control/RevsStamped.h>
#include <indyav_gazebo/wheel_plugin.hpp>

namespace gazebo
{
class BackWheelPlugin : public WheelPlugin<indyav_control::RevsStamped>
{
public:
  BackWheelPlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();
  virtual void Callback(const indyav_control::RevsStamped& _msg);
  virtual void OnUpdate();

protected:
  std::string back_axle_joint_name_;
  physics::JointPtr back_axle_joint_;

  double wheel_rotational_vel_ = 0.0;
  double max_wheel_rotational_vel_ = 0.0;
  double max_velocity_ = 0.0;
};

GZ_REGISTER_MODEL_PLUGIN(BackWheelPlugin)
}
