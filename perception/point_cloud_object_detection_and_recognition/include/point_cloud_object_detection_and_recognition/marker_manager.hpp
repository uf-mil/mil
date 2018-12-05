#pragma once

#include "object_map.hpp"
#include "pcodar_types.hpp"

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <mil_msgs/PerceptionObject.h>
#include <mil_msgs/PerceptionObjectArray.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace pcodar
{
/**
 * Publishes interactive markers visualizing the objects in the database and allowing
 * the user to modify classification in RVIZ.
 */
class MarkerManager
{
public:
  /// Initialize the marker manager with a nodehandle in the namespace of where the markers will be published
  void initialize(ros::NodeHandle& nh, std::shared_ptr<ObjectMap> objects_);
  /// Update the interactive markers
  void update_markers();

private:
  /// Update/Create the interactive marker for one object
  void update_interactive_marker(mil_msgs::PerceptionObject const& object);
  /// Called when user selects new classification in rviz
  void feedbackCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  /// Interactive marker server
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  /// List of possible object classifications from param
  std::vector<std::string> classifications_;
  /// Pointer to PCODAR object map
  std::shared_ptr<ObjectMap> objects_;
};

}  // namespace pcodar
