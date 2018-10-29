#pragma once

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
 * Publishes markers visualizing the objects in the database.
 * @todo: Add interactive markers to set labels from RVIZ
 */
class MarkerManager
{
public:
  /// Initialize the marker manager with a nodehandle in the namespace of where the markers will be published
  void initialize(ros::NodeHandle& nh);
  /// Update the markers based on the latest published objects message
  void update_markers(mil_msgs::PerceptionObjectArray const& objects);

private:
  static void get_object_marker(mil_msgs::PerceptionObject const& object, visualization_msgs::Marker& marker);
  static void get_text_marker(mil_msgs::PerceptionObject const& object, visualization_msgs::Marker& marker);

  ros::Publisher markers_pub_;
};

}  // namespace pcodar
