#include "sub8_gazebo/sub8_thrusters.hpp"
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin)

ThrusterPlugin::ThrusterPlugin()
{
}

void ThrusterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");

  this->model = _model;
  physics::WorldPtr world = _model->GetWorld();
  GZ_ASSERT(world != NULL, "Model is in a NULL world");

  this->physicsEngine = world->Physics();
  GZ_ASSERT(this->physicsEngine != NULL, "Physics engine was NULL");

  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");
  this->sdf = _sdf;

  // Make sure the ROS node for Gazebo has already been initialized
  GZ_ASSERT(ros::isInitialized(), "ROS not initialized");
  thrustSub = nh.subscribe("thrusters/thrust", 1, &ThrusterPlugin::ThrustCallback, this);

  // Parse SDF
  if (this->sdf->HasElement("layout_param"))
  {
    this->layoutParam = this->sdf->Get<std::string>("layout_param");
  }
  else
  {
    GZ_ASSERT(false, "Requires layout_param to be set!");
  }

  if (this->sdf->HasElement("min_abs_thrust"))
  {
    this->minAbsThrust = this->sdf->Get<double>("min_abs_thrust");
  }
  else
  {
    GZ_ASSERT(false, "Requires layout_param to be set!");
  }

  // Apply to multiple links
  GZ_ASSERT(this->sdf->HasElement("link"), "Must have a link specified!");

  std::string link_name = this->sdf->Get<std::string>("link");
  this->targetLink = this->model->GetLink(link_name);
  GZ_ASSERT(this->targetLink, "Could not find specified link");

  // Load thruster layout from parameter server
  XmlRpc::XmlRpcValue thrusterLayout;
  ros::param::get(this->layoutParam, thrusterLayout);
  std::map<std::string, sub8_gazebo::ThrusterDef> thrusterDefs;
  sub8_gazebo::load_thrusters(thrusterLayout["thrusters"], thrusterDefs);

  // Prevent silly warnings (WHO IS STILL USING VARIADIC MACROS!?)
  ROS_INFO("Found %d thrusters", static_cast<int>(thrusterDefs.size()));
  for (const auto& td : thrusterDefs)
  {
    Thruster thruster(td.second);
    thrusterMap[td.first] = thruster;
  }
  this->lastTime = ros::Time::now();
}

void ThrusterPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ThrusterPlugin::OnUpdate, this));
}

void ThrusterPlugin::ThrustCallback(const sub8_msgs::Thrust::ConstPtr& thrust)
{
  ignition::math::Vector3d netForce = ignition::math::Vector3d::Zero;
  ignition::math::Vector3d netTorque = ignition::math::Vector3d::Zero;

  mtx.lock();
  this->lastTime = ros::Time::now();
  this->cmdQueue = thrust->thruster_commands;
  mtx.unlock();
}

void ThrusterPlugin::OnUpdate()
{
  if ((ros::Time::now() - this->lastTime) > ros::Duration(2.0))
  {
    return;
  }
  mtx.lock();
  std::vector<sub8_msgs::ThrusterCmd> cmdBuffer;
  cmdBuffer = this->cmdQueue;
  mtx.unlock();

  GZ_ASSERT(this->targetLink, "Could not find specified link");

  for (auto thrustCmd : cmdBuffer)
  {
    const std::string name = thrustCmd.name;
    const double thrust = thrustCmd.thrust;

    if (std::abs(thrust) < this->minAbsThrust)
    {
      // Apply no force
      continue;
    }

    // Clip thrust within range
    double clipped_thrust = std::max(thrusterMap[name].min, std::min(thrusterMap[name].max, thrust));
    ignition::math::Vector3d force = thrusterMap[name].direction * clipped_thrust;

    targetLink->AddLinkForce(force, thrusterMap[name].position);
  }
}

}  // namespace gazebo
