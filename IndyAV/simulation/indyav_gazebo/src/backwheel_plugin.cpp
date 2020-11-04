#include <indyav_gazebo/backwheel_plugin.hpp>

namespace gazebo
{
BackWheelPlugin::BackWheelPlugin() : WheelPlugin()
{
}

void BackWheelPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  WheelPlugin::Load(_model, _sdf);

  GZ_ASSERT(_sdf->HasElement("max_wheel_rotational_vel"), "no max wheel velocity provided");
  max_wheel_rotational_vel_ = _sdf->Get<double>("max_wheel_rotational_vel");

  GZ_ASSERT(_sdf->HasElement("max_velocity"), "no max velocity provided");
  max_velocity_ = _sdf->Get<double>("max_velocity");

  GZ_ASSERT(_sdf->HasElement("back_axle_joint"), "no back axle provided");
  back_axle_joint_name_ = _sdf->Get<std::string>("back_axle_joint");
  back_axle_joint_ = model_->GetJoint(back_axle_joint_name_);
}

void BackWheelPlugin::Callback(const indyav_control::RevsStamped& _msg)
{
  // TODO: simulate engine input delay
  if (abs(_msg.radians_per_second) <= max_wheel_rotational_vel_)
    wheel_rotational_vel_ = _msg.radians_per_second;
}

void BackWheelPlugin::OnUpdate()
{
  // TODO: apply forces on base_link at the offsets of the wheels in varrying amounts
  //  to replicate a differential torque thingy
  if (model_->RelativeLinearVel().X() < max_velocity_)
  {
    back_axle_joint_->SetVelocity(0, wheel_rotational_vel_);
  }
  else
  {
    back_axle_joint_->SetVelocity(0, max_wheel_rotational_vel_);
  }
}

void BackWheelPlugin::Init()
{
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&BackWheelPlugin::OnUpdate, this));
}
}
