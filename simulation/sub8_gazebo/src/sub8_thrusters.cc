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

  // Apply to multiple links
  GZ_ASSERT(this->sdf->HasElement("link"), "Must have a link specified!");

  std::string link_name = this->sdf->Get<std::string>("link");
  this->targetLink = this->model->GetLink(link_name);
  GZ_ASSERT(this->targetLink, "Could not find specified link");

  // for (sdf::ElementPtr linkElem = this->sdf->GetElement("link"); linkElem;
  //      linkElem = this->sdf->GetNextElement("link")) {
  //   int id = -1;
  //   std::string name = "";
  //   if (linkElem->HasAttribute("name")) {
  //     name = linkElem->Get<std::string>("name");
  //     this->targetLink = this->model->GetLink(name);
  //     GZ_ASSERT(this->targetLink, "Could not find specified link");
  //     id = this->targetLink->GetId();
  //   }
  // }

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

void ThrusterPlugin::ThrustCallback(const sub8_msgs::Thrust::ConstPtr &thrust) {
  math::Vector3 netForce = math::Vector3::Zero;
  math::Vector3 netTorque = math::Vector3::Zero;

  this->cmdQueue.clear();
  this->lastTime = ros::Time::now();
  this->cmdQueue = thrust->thruster_commands;
  // for (auto thrustCmd : thrust->thruster_commands) {
  // this->cmdQueue.push_back(thrustCmd);
  // }
}
void ThrusterPlugin::OnUpdate() {
  if ((ros::Time::now() - this->lastTime) > ros::Duration(2.0)) {
    ROS_ERROR("CLEARING COMMAND QUEUE");
    this->cmdQueue.clear();
  }
  for (auto thrustCmd : this->cmdQueue) {
    const std::string name = thrustCmd.name;
    const double thrust = thrustCmd.thrust;
    const Thruster thruster = this->thrusterMap[name];

    // clamp idea from: https://www.c-plusplus.net/forum/323030-full
    // email me (jake) if you ever find this line
    // math::Vector3 force = thruster.direction * std::max(thruster.min, std::min(thruster.max,
    // thrust));
    math::Vector3 force = thruster.direction * thrust;

    this->targetLink->AddForceAtRelativePosition(force, thruster.position);
  }
}
}
