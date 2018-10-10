#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <point_cloud_object_detection_and_recognition/pcodar_controller.hpp>
#include <point_cloud_object_detection_and_recognition/object.hpp>
#include <map>

namespace mil_gazebo
{

/// Gazebo plugin to pretend to be the PCODAR object database, delivering
class PCODARGazebo : public gazebo::WorldPlugin
{

public:
  /// Load plugin
  void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);
  /// Convert a gazebo math pose to a geometry msgs
  static void GazeboPoseToRosMsg(gazebo::math::Pose const& in, geometry_msgs::Pose& out);
  /// Convert a gazebo math vector to a geometry msg
  static void GazeboVectorToRosMsg(gazebo::math::Vector3 const& in, geometry_msgs::Vector3& out);
  /// TODO
  void UpdateObjects();
  /// TODO
  void UpdateObject(gazebo::physics::ModelPtr _model);
  /// TODO
  void UpdateLink(gazebo::physics::LinkPtr _link);
  /// TODO
  void TimerCb(const ros::TimerEvent&);
private:
  /// Node handle used for ros interactions
  ros::NodeHandle nh_;
  // PCODAR controller
  std::unique_ptr<pcodar::pcodar_controller_base> pcodar_;
  /// Map of gazebo model/links to what they should appear as in database
  std::map<std::string, std::string> name_map_;
  /// Pointer to the gazebo world were these models are
  gazebo::physics::WorldPtr world_;
  /// Timer to update ogrid, markers
  ros::Timer timer_;
};

}
