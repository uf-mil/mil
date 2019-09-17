#ifndef _GAZEBO_STATE_PLUGIN_HH_
#define _GAZEBO_STATE_PLUGIN_HH_

#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"

#include <geometry_msgs/PoseStamped.h>
#include "ros/ros.h"

namespace gazebo
{
class StatePlugin : public ModelPlugin
{
public:
  StatePlugin();
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();

protected:
  virtual void OnUpdate();
  virtual void PoseRefUpdate(const geometry_msgs::PoseStampedConstPtr& ps);
  event::ConnectionPtr update_connection_;
  physics::ModelPtr model_;
  physics::PhysicsEnginePtr physics_engine_;
  sdf::ElementPtr sdf_;

  ros::NodeHandle nh_;
  ros::Subscriber reference_sub_;
  ignition::math::Vector3d static_offset_;
  ignition::math::Vector3d modelOffset;  // Offset from the (0,0,0) point in the model to base_link
  ignition::math::Pose3d first_pose_;     // Starting position of model, to add to each reference
  ignition::math::Pose3d last_ref_pose_;
};
}

#endif
