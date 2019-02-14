
#ifndef MIL_MAGNETOMETER_GAZEBO_HPP
#define MIL_MAGNETOMETER_GAZEBO_HPP

#include <sensor_msgs/MagneticField.h>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

namespace mil_gazebo
{
/// \brief Plugin to simulate a DVL
class MilMagnetometerGazebo : public gazebo::SensorPlugin
{
  /// Constructor
public:
  MilMagnetometerGazebo();

  /// Destructor
public:
  ~MilMagnetometerGazebo();

  /// \brief Load the plugin
  /// \param take in SDF root element
public:
  void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  /// Update the controller
private:
  void OnUpdate();
  ros::NodeHandle nh_;
  ros::Publisher mag_pub_;
  gazebo::sensors::MagnetometerSensorPtr sensor_;
  std::string frame_name_;
  gazebo::event::ConnectionPtr update_connection_;
};
}

#endif
