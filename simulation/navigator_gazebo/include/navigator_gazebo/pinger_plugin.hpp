#pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <vector>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/physics.hh"

namespace navigator_gazebo
{
class PingerPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  bool ServiceCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  std::string NewRandomGate();
  ros::NodeHandle nh_;
  ros::ServiceServer service_;
  gazebo::physics::ModelPtr model_;
};
}
