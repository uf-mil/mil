#include <point_cloud_object_detection_and_recognition/marker_manager.hpp>

// #include <ros/console.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <tf/transform_datatypes.h>

namespace pcodar
{
void MarkerManager::initialize(ros::NodeHandle& nh, std::shared_ptr<ObjectMap> _objects)
{
  this->objects_ = _objects;
  nh.getParam("classifications", classifications_);
  interactive_marker_server_.reset(new interactive_markers::InteractiveMarkerServer("pcodar_objects", "", false));
}

void MarkerManager::feedbackCb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // Ignore feedback that isn't a new menu option selected
  if ((*feedback).event_type != 2)
    return;

  // Get the corresponding classification string based on the index of the menu option clicked
  size_t classification_idx = (*feedback).menu_entry_id - 1;
  if (classification_idx >= classifications_.size())
    return;
  std::string new_classification = classifications_.at(classification_idx);

  // Find the object with this name in the database
  int id = std::stoi((*feedback).marker_name.substr(6));
  auto it = objects_->objects_.find(id);
  if (it == objects_->objects_.end())
    return;

  // Update the classification
  (*it).second.msg_.labeled_classification = new_classification;
}

void MarkerManager::update_interactive_marker(mil_msgs::PerceptionObject const& object)
{
  // Form the marker name from the object id
  std::string name = "object" + std::to_string(object.id);

  // Stores the new / pervious interactive marker
  visualization_msgs::InteractiveMarker int_marker;

  // Tracks if the marker needs to be inserted or just updated
  bool is_new = false;

  // If marker is not already added for this object, create a new one
  if (!interactive_marker_server_->get(name, int_marker))
  {
    is_new = true;

    // Create new interactive marker for the object
    int_marker.header.frame_id = "enu";
    int_marker.scale = 1.;
    int_marker.name = name;

    // Add a menu option for each classification
    size_t option_id = 1;
    for (std::string const& classification : classifications_)
    {
      visualization_msgs::MenuEntry option;
      option.id = option_id++;
      option.parent_id = 0;
      option.title = classification;
      int_marker.menu_entries.push_back(option);
    }

    // Add right clock control to bring up menu
    visualization_msgs::InteractiveMarkerControl control;
    control.name = name + " Control";
    control.description = control.name;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    control.always_visible = true;

    // Create bounding box marker
    visualization_msgs::Marker box_marker;
    box_marker.type = visualization_msgs::Marker::CUBE;
    box_marker.id = 10000 + object.id;
    box_marker.color.a = 0.5;
    box_marker.color.r = 1.0;
    box_marker.color.g = 1.0;
    box_marker.color.b = 1.0;
    control.markers.push_back(box_marker);

    // Create text with object id + classification
    visualization_msgs::Marker text_marker;
    text_marker.id = 20000 + object.id;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = 1;
    text_marker.color.a = 0.7;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    control.markers.push_back(text_marker);

    int_marker.controls.push_back(control);
  }

  // Update newly created / existing object with new stamp, pose, box, and classification
  // TODO: only update if pose / box / classification has changed
  int_marker.header.stamp = ros::Time::now();
  int_marker.pose = object.pose;
  if (int_marker.pose.position.z < 0.)
    int_marker.pose.position.z = 0.;
  int_marker.controls[0].markers[0].scale.x = object.scale.x + 0.5;
  int_marker.controls[0].markers[0].scale.y = object.scale.y + 0.5;
  int_marker.controls[0].markers[0].scale.z = object.scale.z + 0.5;
  int_marker.controls[0].markers[1].text =
      object.classification + " | " + object.labeled_classification + " (" + std::to_string(object.id) + ")";

  // If marker is new, insert it with the callback
  if (is_new)
    interactive_marker_server_->insert(int_marker, std::bind(&MarkerManager::feedbackCb, this, std::placeholders::_1));
  // Otherwise, insert without callback to update existing marker
  else
    interactive_marker_server_->insert(int_marker);
}

void MarkerManager::update_markers()
{
  // Update / add markers for each object in database
  for (const auto& pair : objects_->objects_)
  {
    mil_msgs::PerceptionObject const& msg = pair.second.msg_;
    update_interactive_marker(msg);
  }

  // Publish marker changes to RVIZ clients
  interactive_marker_server_->applyChanges();
}

}  // pcodar namespace
