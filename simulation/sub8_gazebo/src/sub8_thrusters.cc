#include "sub8_gazebo/sub8_thrusters.hpp"
#include <gazebo/common/Plugin.hh>

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(ThrusterPlugin)

ThrusterPlugin::ThrusterPlugin() {}

void ThrusterPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");

  this->model = _model;
  physics::WorldPtr world = _model->GetWorld();
  GZ_ASSERT(world != NULL, "Model is in a NULL world");

  this->physicsEngine = world->GetPhysicsEngine();
  GZ_ASSERT(this->physicsEngine != NULL, "Physics engine was NULL");

  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");
  this->sdf = _sdf;

  // Make sure the ROS node for Gazebo has already been initialized
  GZ_ASSERT(ros::isInitialized(), "ROS not initialized");
  thrustSub = nh.subscribe("thrusters/thrust", 1, &ThrusterPlugin::ThrustCallback, this);

  // Parse SDF
  if (this->sdf->HasElement("layout_param")) {
    this->layoutParam = this->sdf->Get<std::string>("layout_param");
  } else {
    GZ_ASSERT(false, "Requires layout_param to be set!");
  }

  if (this->sdf->HasElement("min_abs_thrust")) {
    this->minAbsThrust = this->sdf->Get<double>("min_abs_thrust");
  } else {
    GZ_ASSERT(false, "Requires layout_param to be set!");
  }


  // Apply to multiple links
  GZ_ASSERT(this->sdf->HasElement("link"), "Must have a link specified!");

  std::string link_name = this->sdf->Get<std::string>("link");
  this->targetLink = this->model->GetLink(link_name);
  GZ_ASSERT(this->targetLink, "Could not find specified link");

  std::vector<std::string> thrusterNames;
  ros::param::get(this->layoutParam + "/thruster_names", thrusterNames);

  // Prevent silly warnings (WHO IS STILL USING VARIADIC MACROS!?)
  ROS_INFO("Found %d thrusters", static_cast<int>(thrusterNames.size()));
  for (std::string thrusterName : thrusterNames) {
    Thruster thruster(layoutParam + "/" + thrusterName);
    thrusterMap[thrusterName] = thruster;
  }
  this->lastTime = ros::Time::now();
}

void ThrusterPlugin::Init() {
  this->updateConnection =
      event::Events::ConnectWorldUpdateBegin(std::bind(&ThrusterPlugin::OnUpdate, this));
}

void ThrusterPlugin::ThrustCallback(const sub8_msgs::Thrust::ConstPtr& thrust) {
  math::Vector3 netForce = math::Vector3::Zero;
  math::Vector3 netTorque = math::Vector3::Zero;

  mtx.lock();
  this->lastTime = ros::Time::now();
  this->cmdQueue = thrust->thruster_commands;
  mtx.unlock();
}

void ThrusterPlugin::OnUpdate() {
  if ((ros::Time::now() - this->lastTime) > ros::Duration(2.0)) {
    return;
  }
  mtx.lock();
  std::vector<sub8_msgs::ThrusterCmd> cmdBuffer;
  cmdBuffer = this->cmdQueue;
  mtx.unlock();

  GZ_ASSERT(this->targetLink, "Could not find specified link");

  math::Vector3 net_force(0.0, 0.0, 0.0);
  math::Vector3 net_torque(0.0, 0.0, 0.0);

  for (auto thrustCmd : cmdBuffer) {
    const std::string name = thrustCmd.name;
    const double thrust = thrustCmd.thrust;

    if (std::abs(thrust) < this->minAbsThrust) {
      // Apply no force
      continue;
    }

    // clamp idea from: https://www.c-plusplus.net/forum/323030-full
    // email me (jake) if you ever find this line
    math::Vector3 force = thrusterMap[name].direction *
                          std::max(thrusterMap[name].min, std::min(thrusterMap[name].max, thrust));
    math::Vector3 torque = thrusterMap[name].position.Cross(force);
    net_force += force;
    net_torque += torque;

    // math::Vector3 force;
    // force = thrusterMap[name].direction * thrust;
  }
  // this->targetLink->AddForceAtRelativePosition(force, thrusterMap[name].position);
  math::Pose subFrame = this->targetLink->GetWorldPose();
  if (subFrame.pos.z < 0.0) {
    this->targetLink->AddRelativeForce(net_force);
    this->targetLink->AddRelativeTorque(net_torque);
  }
}
}
