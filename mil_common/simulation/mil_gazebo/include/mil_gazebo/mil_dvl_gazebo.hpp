
#ifndef MIL_DVL_GAZEBO_HPP
#define MIL_DVL_GAZEBO_HPP

#include <mil_msgs/RangeStamped.h>
#include <mil_msgs/VelocityMeasurements.h>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

namespace mil_gazebo
{
/// \brief Plugin to simulate a DVL
class MilDVLGazebo : public gazebo::SensorPlugin
{
  /// Constructor
public:
  MilDVLGazebo();

  /// Destructor
public:
  ~MilDVLGazebo();

  /// \brief Load the plugin
  /// \param take in SDF root element
public:
  void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  /// Update the controller
private:
  void OnUpdate();

  /// \brief pointer to ros node
private:
  ros::NodeHandle nh_;

private:
  ros::Publisher vel_pub_;

private:
  ros::Publisher range_pub_;

private:
  gazebo::sensors::RaySensorPtr sensor_;

private:
  ignition::math::Pose3d pose_;

private:
  ignition::math::Vector3d offset_;

private:
  std::string frame_name_;

private:
  gazebo::physics::LinkPtr parent_;

  // Pointer to the update event connection
private:
  gazebo::event::ConnectionPtr update_connection_;
};
}

#endif
