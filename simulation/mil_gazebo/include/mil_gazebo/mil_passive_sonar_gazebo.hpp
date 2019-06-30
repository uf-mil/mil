#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

namespace mil_gazebo
{
/// Gazebo plugin to publish a heading from one object to another
class MilPassiveSonarGazebo : public gazebo::ModelPlugin
{
public:
  /// Load plugin
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);
  ///
  void TimerCb(const ros::TimerEvent&);
  /// Convert a gazebo math vector to a geometry msg
  static void GazeboVectorToRosMsg(gazebo::math::Vector3 const& in, geometry_msgs::Vector3& out);

private:
  /// Node handle used for ros interactions
  ros::NodeHandle nh_;
  /// Publish to publish vector
  ros::Publisher vector_pub_;
  /// Publish processed ping
  ros::Publisher processed_ping_pub_;
  /// Frame of point to publish from
  std::string frame_;
  /// Offset from model to where origin is
  gazebo::math::Pose offset_;
  /// Pointer to the gazebo world were these models are
  gazebo::physics::ModelPtr origin_;
  /// Pointer to model to get heading to
  gazebo::physics::ModelPtr model_;
  /// Timer to update ogrid, markers
  ros::Timer timer_;
  // Frequency
  double freq_ = 37000;
  // Amplitude
  // This default value is entirely guessed at
  double amplitude_ = 1000;
};
}
