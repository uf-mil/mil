#include <mil_gazebo/mil_drag_gazebo.hpp>

namespace mil_gazebo
{
// Register this plugin
GZ_REGISTER_MODEL_PLUGIN(MilDragGazebo)

MilDragGazebo::MilDragGazebo()
{
}

MilDragGazebo::~MilDragGazebo()
{
}

void MilDragGazebo::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  model_ = _parent;
  if (!model_)
  {
    gzerr << "parent model is null";
    return;
  }

  link_ = model_->GetLink();
  if (!link_)
  {
    gzerr << "link is null";
    return;
  }

  if (!_sdf->HasElement("linear_coeffs"))
  {
    gzerr << "missing linear coeffs param";
  }
  linear_coeffs_ = _sdf->GetElement("linear_coeffs")->Get<gazebo::math::Vector3>();

  if (!_sdf->HasElement("angular_coeffs"))
  {
    gzerr << "missing linear coeffs param";
  }
  angular_coeffs_ = _sdf->GetElement("angular_coeffs")->Get<gazebo::math::Vector3>();

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MilDragGazebo::OnUpdate, this, std::placeholders::_1));
}

void MilDragGazebo::OnUpdate(const gazebo::common::UpdateInfo& info)
{
  gazebo::math::Vector3 linear_vel = link_->GetRelativeLinearVel();
  gazebo::math::Vector3 angular_vel = link_->GetRelativeAngularVel();

  gazebo::math::Vector3 force = -linear_coeffs_ * linear_vel;
  gazebo::math::Vector3 torque = -angular_coeffs_ * angular_vel;

/*
  gzerr << "Vel " << linear_vel
        << "Ang " << angular_vel
        << "Force " << force
        << "Torque " << torque;
*/

  link_->AddRelativeForce(force);
  link_->AddRelativeTorque(torque);
}

}
