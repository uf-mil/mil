#include <indyav_gazebo/backwheel_plugin.hpp>

namespace gazebo
{
BackWheelPlugin::BackWheelPlugin() : WheelPlugin()
{
}

void BackWheelPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  WheelPlugin::Load(_model, _sdf);

  GZ_ASSERT(_sdf->HasElement("max_thrust"), "no max thrust provided");
  max_thrust_ = _sdf->Get<double>("max_thrust");

  GZ_ASSERT(_sdf->HasElement("max_velocity"), "no max velocity provided");
  max_velocity_ = _sdf->Get<double>("max_velocity");

  base_link_ = model_->GetLink("base_link");
}

void BackWheelPlugin::Callback(const indyav_control::ThrustStamped& _msg)
{
  // TODO: simulate engine input delay
  if (abs(_msg.thrust) <= max_thrust_)
    thrust_.X(_msg.thrust);
}

void BackWheelPlugin::OnUpdate()
{
  // TODO: only apply forces when wheel collisions are touching the ground
  // TODO: apply forces on base_link at the offsets of the wheels in varrying amounts
  //  to replicate a differential torque thingy
  if (model_->RelativeLinearVel().X() < max_velocity_)
    base_link_->AddLinkForce(thrust_, ignition::math::Vector3<double>::Zero);
}

void BackWheelPlugin::Init()
{
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&BackWheelPlugin::OnUpdate, this));
}
}
