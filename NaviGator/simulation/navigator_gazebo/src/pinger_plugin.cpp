#include "navigator_gazebo/pinger_plugin.hpp"
#include <gazebo/common/Plugin.hh>

namespace navigator_gazebo
{
GZ_REGISTER_MODEL_PLUGIN(PingerPlugin)

void PingerPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  nh_ = ros::NodeHandle("navigator_pinger_plugin");
  service_ = nh_.advertiseService("/hydrophones/switch_gate", &PingerPlugin::ServiceCallback, this);
  NewRandomGate();
}

bool PingerPlugin::ServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  std::string new_gate = NewRandomGate();
  if (new_gate.empty())
  {
    res.success = false;
    res.message = "could not set gate, see ros error";
    return true;
  }
  res.success = true;
  res.message = "New Gate: " + new_gate;
  return true;
}

std::string PingerPlugin::NewRandomGate()
{
  ignition::math::Vector3d gates[3];

  gazebo::physics::ModelPtr entrance_gate = model_->GetWorld()->ModelByName("robotx_2018_entrance_gate");
  if (!entrance_gate)
  {
    ROS_ERROR_NAMED("pinger_plugin", "Entrance gate model not found.");
    return "";
  }

  gazebo::physics::LinkPtr markers[4];
  markers[0] = entrance_gate->GetLink("base_gate::red_buoy::link");
  markers[1] = entrance_gate->GetLink("base_gate::white_buoy_left::link");
  markers[2] = entrance_gate->GetLink("base_gate::white_buoy_right::link");
  markers[3] = entrance_gate->GetLink("base_gate::green_buoy::link");

  for (size_t i = 0; i < 4; ++i)
  {
    if (markers[i])
      continue;
    ROS_ERROR_NAMED("pinger_plugin", "Marker %lu not found", i);
    return "";
  }

  for (size_t i = 0; i < 3; ++i)
  {
    gates[i] = (markers[i]->WorldPose().Pos() + markers[i + 1]->WorldPose().Pos()) / 2.0;
    gates[i].Z(-5.0);
  }

  int random_selection = ignition::math::Rand::IntUniform(0, 2);
  ignition::math::Pose3d new_pose(gates[random_selection], ignition::math::Quaterniond());

  model_->SetWorldPose(new_pose);

  return std::to_string(random_selection + 1);
}

}  // namespace navigator_gazebo
