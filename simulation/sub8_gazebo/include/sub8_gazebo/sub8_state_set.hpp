#ifndef _GAZEBO_STATE_PLUGIN_HH_
#define _GAZEBO_STATE_PLUGIN_HH_

#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/physics/physics.hh"

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

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
      math::Vector3 static_offset_;
      math::Vector3 modelOffset;  // Offset from the (0,0,0) point in the model to base_link
      math::Pose first_pose_;  // Starting position of model, to add to each reference
      math::Pose last_ref_pose_;

  };
}

#endif
