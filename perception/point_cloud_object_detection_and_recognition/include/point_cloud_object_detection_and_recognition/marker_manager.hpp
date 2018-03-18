#pragma once

#include "pcodar_types.hpp"
#include "pcodar_params.hpp"


#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <mil_msgs/PerceptionObject.h>

#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>

namespace pcodar
{
class marker_manager
{
   public:
    void initialize(ros::NodeHandle& nh, id_label_map_ptr id_label_map);

    void update_markers(const id_object_map_ptr objects);
    
    visualization_msgs::Marker get_marker_object(const mil_msgs::PerceptionObject& object);
    visualization_msgs::Marker get_marker_text(const mil_msgs::PerceptionObject& object);
	visualization_msgs::InteractiveMarker get_marker_interactive(const mil_msgs::PerceptionObject& object);

   private:
    ros::Publisher pub_markers_objects_;
    ros::Publisher pub_markers_text_;
    id_label_map_ptr id_label_map_;

    std::unique_ptr<interactive_markers::InteractiveMarkerServer> interactive_marker_server_;
    interactive_markers::MenuHandler menu_handler_;
    interactive_markers::MenuHandler::EntryHandle menu_entry_;
    void interative_marker_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
};
}
