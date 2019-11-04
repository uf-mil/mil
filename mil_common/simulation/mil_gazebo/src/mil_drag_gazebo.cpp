#include <ros/ros.h>
#include <mil_gazebo/mil_drag_gazebo.hpp>
#include <mil_gazebo/mil_gazebo_utils.hpp>

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

  if (!GetFromSDFOrRosParam(_sdf, "linear_coeffs", linear_coeffs_))
  {
    ROS_ERROR("linear_coeffs_bad");
    return;
  }

  if (!GetFromSDFOrRosParam(_sdf, "angular_coeffs", angular_coeffs_))
  {
    ROS_ERROR("angular_coeffs_bad");
    return;
  }

  update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MilDragGazebo::OnUpdate, this, std::placeholders::_1));
}

void MilDragGazebo::OnUpdate(const gazebo::common::UpdateInfo& info)
{
  ignition::math::Vector3d linear_vel = link_->RelativeLinearVel();
  ignition::math::Vector3d angular_vel = link_->RelativeAngularVel();

  ignition::math::Vector3d force = -linear_coeffs_ * linear_vel;
  ignition::math::Vector3d torque = -angular_coeffs_ * angular_vel;

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
