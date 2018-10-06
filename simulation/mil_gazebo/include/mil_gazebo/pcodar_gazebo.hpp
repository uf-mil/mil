#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <mil_msgs/ObjectDBQuery.h>
#include <map>

namespace mil_gazebo
{

/// Gazebo plugin to pretend to be the PCODAR object database, delivering
class PCODARGazebo : public gazebo::WorldPlugin
{

public:
  /// Load plugin
  void Load(gazebo::physics::WorldPtr _parent, sdf::ElementPtr _sdf); 
  /// Callback for service requests to database
  bool DatabaseQuery(mil_msgs::ObjectDBQuery::Request &req, mil_msgs::ObjectDBQuery::Response& res);
  /// Add an object to a database query request if it matches the query
  void AddDatabaseObject(mil_msgs::ObjectDBQuery::Request &req, mil_msgs::ObjectDBQuery::Response& res, gazebo::physics::ModelPtr _model);
  /// Add a link t othe database query if it matches the query
  void AddDatabaseLink(mil_msgs::ObjectDBQuery::Request &req, mil_msgs::ObjectDBQuery::Response& res, gazebo::physics::LinkPtr _link);
  /// Convert a gazebo math pose to a geometry msgs
  static void GazeboPoseToRosMsg(gazebo::math::Pose const& in, geometry_msgs::Pose& out);
private:
  /// Node handle used for ros interactions
  ros::NodeHandle nh_;
  /// Service to simulate PCODAR database
  ros::ServiceServer server_;
  /// Map of gazebo model/links to what they should appear as in database
  std::map<std::string, std::string> name_map_;
  /// Pointer to the gazebo world were these models are
  gazebo::physics::WorldPtr world_;
};

}
