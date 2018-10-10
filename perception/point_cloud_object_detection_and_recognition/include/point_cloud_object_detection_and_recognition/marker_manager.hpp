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
class marker_manager
{
public:
  void initialize(ros::NodeHandle& nh);

  void update_markers(mil_msgs::PerceptionObjectArray const& objects);

  visualization_msgs::InteractiveMarker get_marker_interactive(const mil_msgs::PerceptionObject& object);

private:
  static void get_object_marker(mil_msgs::PerceptionObject const& object, visualization_msgs::Marker& marker);
  static void get_text_marker(mil_msgs::PerceptionObject const& object, visualization_msgs::Marker& marker);

  void interative_marker_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);

  ros::Publisher markers_pub_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
  interactive_markers::MenuHandler menu_handler_;
  interactive_markers::MenuHandler::EntryHandle menu_entry_;
};

}  // namespace pcodar
