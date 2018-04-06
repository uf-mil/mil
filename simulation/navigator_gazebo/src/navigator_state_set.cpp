#include "navigator_gazebo/navigator_state_set.hpp"
#include <sstream>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "ros/ros.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(StatePlugin)

/////////////////////////////////////////////////
StatePlugin::StatePlugin() : pose(0.0, 0.0, 1.2, 0.0, 0.0, 0.0)
{
}

/////////////////////////////////////////////////
void StatePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");
  this->model = _model;
  physics::WorldPtr world = _model->GetWorld();
  GZ_ASSERT(world != NULL, "Model is in a NULL world");
  this->physicsEngine = world->GetPhysicsEngine();
  GZ_ASSERT(this->physicsEngine != NULL, "Physics engine was NULL");

  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");
  this->sdf = _sdf;

  if (this->sdf->HasElement("model_offset"))
  {
    this->modelOffset = this->sdf->Get<math::Vector3>("model_offset");
  }
  else
  {
    this->modelOffset = math::Vector3(0.0, 0.0, 0.0);
  }

  // Make sure the ROS node for Gazebo has already been initialized
  GZ_ASSERT(ros::isInitialized(), "ROS not initialized");
  this->refSub = nh.subscribe("/trajectory/cmd", 1, &StatePlugin::PoseRefUpdate, this);
}
/////////////////////////////////////////////////
void StatePlugin::PoseRefUpdate(const nav_msgs::OdometryConstPtr& odom)
{
  math::Vector3 pos(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
  math::Quaternion rot(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
                       odom->pose.pose.orientation.z);

  this->pose = math::Pose(pos + rot.RotateVector(this->modelOffset), rot);
}

/////////////////////////////////////////////////
void StatePlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&StatePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void StatePlugin::OnUpdate()
{
  this->model->SetWorldPose(this->pose);
}
