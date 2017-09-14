#ifndef _GAZEBO_STATE_PLUGIN_HH_
#define _GAZEBO_STATE_PLUGIN_HH_

#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/physics.hh"

#include "nav_msgs/Odometry.h"
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
  virtual void PoseRefUpdate(const nav_msgs::OdometryConstPtr& odom);
  event::ConnectionPtr updateConnection;
  physics::ModelPtr model;
  physics::PhysicsEnginePtr physicsEngine;
  sdf::ElementPtr sdf;

  ros::NodeHandle nh;
  ros::Subscriber refSub;
  math::Vector3 modelOffset;  // Offset from the (0,0,0) point in the model to base_link
  math::Pose pose;
};
}

#endif
