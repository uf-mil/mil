#include "navigator_gazebo/navigator_thrusters.hpp"
#include <gazebo/common/Plugin.hh>

namespace navigator_gazebo
{
GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin)

Thruster::Thruster(std::string _name, gazebo::physics::LinkPtr _link, gazebo::physics::JointPtr _joint)
  : link_(_link), joint_(_joint), nh_(_name + "_motor")
{
  command_sub_ = nh_.subscribe("cmd", 1, &Thruster::CommandCallback, this);
}

void Thruster::CommandCallback(const roboteq_msgs::Command& _cmd)
{
  std::lock_guard<std::mutex> lock(mutex_);
  command_ = _cmd.setpoint;
  last_command_time_ = ros::Time::now();
}

void Thruster::Update(double& _position, double& _velocity, double& _effort)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (ros::Time::now() - last_command_time_ > ros::Duration(1))
    command_ = 0.;
  joint_->SetForce(0, command_);
  _effort = command_;
  _position = joint_->GetAngle(0).Radian();
  _velocity = joint_->GetVelocity(0);
  gazebo::math::Vector3 force_vector(command_, 0., 0.);
  link_->AddLinkForce(force_vector);
  // SET LINK FORCE
}

ThrusterPlugin::ThrusterPlugin()
{
}

void ThrusterPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");
  GZ_ASSERT(ros::isInitialized(), "ROS not initialized");

  for (auto link_sdf = _sdf->GetElement("thruster"); link_sdf != nullptr;
       link_sdf = link_sdf->GetNextElement("thruster"))
  {
    std::string name = link_sdf->Get<std::string>();
    std::string link_name = name + "_propeller_link";
    std::string joint_name = name + "_engine_propeller_joint";
    auto link = _model->GetLink(link_name);
    if (!link)
    {
      ROS_ERROR("Link %s not found.", link_name.c_str());
    }
    auto joint = _model->GetJoint(joint_name);
    if (!joint)
    {
      ROS_ERROR("Link %s not found.", joint_name.c_str());
    }
    std::unique_ptr<Thruster> thruster(new Thruster(name, link, joint));
    thrusters_.push_back(std::move(thruster));
    joint_state_msg_.name.push_back(joint_name);
    joint_state_msg_.position.push_back(0.);
    joint_state_msg_.velocity.push_back(0.);
    joint_state_msg_.effort.push_back(0.);
    // Create a thruster object and pushback
  }

  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 10);

  update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ThrusterPlugin::OnUpdate, this, std::placeholders::_1));
}

void ThrusterPlugin::OnUpdate(const gazebo::common::UpdateInfo& info)
{
  for (size_t i = 0; i < thrusters_.size(); ++i)
  {
    thrusters_[i]->Update(joint_state_msg_.position[i], joint_state_msg_.velocity[i], joint_state_msg_.effort[i]);
  }
  joint_state_msg_.header.stamp.sec = info.simTime.sec;
  joint_state_msg_.header.stamp.nsec = info.simTime.nsec;
  joint_state_pub_.publish(joint_state_msg_);
}
}
