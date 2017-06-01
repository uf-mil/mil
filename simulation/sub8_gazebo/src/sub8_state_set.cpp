#include "sub8_gazebo/sub8_state_set.hpp"
#include <sstream>
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "ros/ros.h"

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(StatePlugin)

#include <fstream>
#include <iostream>

/**
 * Gazebo ROS plugin to continuously update a models poition to the
 * position outputed by a trajectory generator
 *
 * Assumes a topic specified as referenceTopic outputs geometry_msgs/PoseStamped
 * where the pose is relative to the starting position of the model in the gazebo world.
 * As each message arrive, gazebo is updated to follow this carrot exactly.
 * This is useful for situations where it is desireable to test perception or other code
 * where physics simulated controls would just by a pain.
 */

/////////////////////////////////////////////////
StatePlugin::StatePlugin()
{
}

/////////////////////////////////////////////////
void StatePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");
  model_ = _model;
  physics::WorldPtr world = _model->GetWorld();
  GZ_ASSERT(world != NULL, "Model is in a NULL world");
  physics_engine_ = world->GetPhysicsEngine();
  GZ_ASSERT(physics_engine_ != NULL, "Physics engine was NULL");

  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");
  sdf_ = _sdf;

  // If model is not center frame, store static offset
  if (sdf_->HasElement("staticOffset"))
  {
    static_offset_ = sdf_->Get<math::Vector3>("staticOffset");
  }
  else
  {
    static_offset_ = math::Vector3(0.0, 0.0, 0.0);
  }

  // Get PoseStamped trajectory
  std::string ref_topic = "/trajectory";
  if (sdf_->HasElement("referenceTopic"))
    ref_topic = sdf_->Get<std::string>("referenceTopic");
  else
    ROS_WARN("SDF does not have 'referenceTopic' element, using default /trajectory topic");

  first_pose_ = model_->GetWorldPose();  // Init pose
  last_ref_pose_.pos.z = first_pose_.pos.z;
  first_pose_.pos.z = 0.0;

  // Make sure the ROS node for Gazebo has already been initialized
  GZ_ASSERT(ros::isInitialized(), "ROS not initialized");
  reference_sub_ = nh_.subscribe(ref_topic, 1, &StatePlugin::PoseRefUpdate, this);
}
/////////////////////////////////////////////////
void StatePlugin::PoseRefUpdate(const geometry_msgs::PoseStampedConstPtr& ps)
{
  math::Vector3 pos(ps->pose.position.x, ps->pose.position.y, ps->pose.position.z);
  math::Quaternion rot(ps->pose.orientation.w, ps->pose.orientation.x, ps->pose.orientation.y, ps->pose.orientation.z);

  last_ref_pose_ = math::Pose(pos, rot);
}

/////////////////////////////////////////////////
void StatePlugin::Init()
{
  update_connection_ = event::Events::ConnectWorldUpdateBegin(std::bind(&StatePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void StatePlugin::OnUpdate()
{
  auto fixed_pose = last_ref_pose_ + first_pose_;
  model_->SetWorldPose(fixed_pose);
}
