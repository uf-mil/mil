#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <map>
#include <point_cloud_object_detection_and_recognition/object.hpp>
#include <point_cloud_object_detection_and_recognition/pcodar_controller.hpp>
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"

namespace mil_gazebo
{
/// Gazebo plugin to pretend to be the PCODAR object database, delivering
class PCODARGazebo : public gazebo::WorldPlugin
{
public:
  /// Load plugin
  void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf);
  /// Convert a gazebo math pose to a geometry msgs
  /// TODO: put into a common, non-PCODAR related library
  static void GazeboPoseToRosMsg(gazebo::math::Pose const& in, geometry_msgs::Pose& out);
  /// Convert a gazebo math vector to a geometry msg
  /// TODO: put into a common, non-PCODAR related library
  static void GazeboVectorToRosMsg(gazebo::math::Vector3 const& in, geometry_msgs::Vector3& out);
  /// Add gazebo objects with matching names to PCODAR
  void UpdateEntities();
  /// Add / Update an entity in PCODAR if it matches one of the selected names
  void UpdateEntity(gazebo::physics::EntityPtr _entity);
  /// Call UpdateEntity on a gazebo model, its submodels, and its links
  void UpdateModel(gazebo::physics::ModelPtr _model);
  /// Called regularly to publish latest PCDOAR info
  void TimerCb(const ros::TimerEvent&);

private:
  /// Node handle used for ros interactions
  ros::NodeHandle nh_;
  // PCODAR controller
  std::unique_ptr<pcodar::NodeBase> pcodar_;
  /// Map of gazebo model/links to what they should appear as in database
  std::map<std::string, std::string> name_map_;
  /// Pointer to the gazebo world were these models are
  gazebo::physics::WorldPtr world_;
  /// Timer to update ogrid, markers
  ros::Timer timer_;
};
}
