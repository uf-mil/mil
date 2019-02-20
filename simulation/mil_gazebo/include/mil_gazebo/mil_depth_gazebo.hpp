
#ifndef MIL_DEPTH_GAZEBO_HPP
#define MIL_DEPTH_GAZEBO_HPP

#include <mil_msgs/DepthStamped.h>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace mil_gazebo
{
/// \brief Plugin to simulate a DVL
class MilDepthGazebo : public gazebo::ModelPlugin
{
  /// Constructor
public:
  MilDepthGazebo();

  /// Destructor
public:
  ~MilDepthGazebo();

  /// \brief Load the plugin
  /// \param take in SDF root element
public:
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

private:
  void OnUpdate(const gazebo::common::UpdateInfo& info);

  gazebo::common::Time last_pub_time_;

  double update_period_ = 1. / 10.;
  ros::NodeHandle nh_;
  ros::Publisher depth_pub_;
  ignition::math::Pose3d offset_;
  gazebo::physics::ModelPtr model_;
  std::string frame_name_;
  gazebo::event::ConnectionPtr update_connection_;
};
}

#endif
