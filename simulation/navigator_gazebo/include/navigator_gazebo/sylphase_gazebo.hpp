#pragma once

#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "roboteq_msgs/Command.h"

namespace navigator_gazebo
{
class SylphaseGazebo : public gazebo::ModelPlugin
{
public:
  SylphaseGazebo();
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void OnUpdate(const gazebo::common::UpdateInfo &info);

private:
  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Publisher absodom_pub_;
  ros::Publisher acceleration_pub_;
  gazebo::physics::ModelPtr model_;
  gazebo::common::SphericalCoordinatesPtr converter_;
  gazebo::event::ConnectionPtr update_connection_;
  std::string child_frame_;
  double update_period_;
  gazebo::common::Time last_update_time_;

  geometry_msgs::Point Convert(const ignition::math::Vector3d &);
  geometry_msgs::Pose Convert(const ignition::math::Pose3d &);
  geometry_msgs::Vector3 Convert2(const ignition::math::Vector3d &);
  geometry_msgs::Quaternion Convert(const ignition::math::Quaterniond &);

  ignition::math::Pose3d pose_;
  ignition::math::Matrix4d local_to_enu_;
  ignition::math::Matrix4d local_to_ecef_;

  using CoordinateType = gazebo::common::SphericalCoordinates::CoordinateType;
  static ignition::math::Matrix4d SphericalCoordinatesTransform(const gazebo::common::SphericalCoordinates &_convert,
                                                                const CoordinateType in, const CoordinateType out);
};
}
