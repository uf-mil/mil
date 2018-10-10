#include <point_cloud_object_detection_and_recognition/marker_manager.hpp>

// #include <ros/console.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <tf/transform_datatypes.h>

namespace pcodar
{
void marker_manager::initialize(ros::NodeHandle& nh, id_label_map_ptr id_label_map)
{
  ros::NodeHandle private_nh("~");
  markers_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>("objects_visualization", 10);
  interactive_marker_server_ = std::unique_ptr<interactive_markers::InteractiveMarkerServer>(
      new interactive_markers::InteractiveMarkerServer("interactive", "", false));
}

void marker_manager::interative_marker_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  ROS_INFO("INTERACTIVE CALLBACK");
}

void marker_manager::get_object_marker(mil_msgs::PerceptionObject const& object, visualization_msgs::Marker& marker)
{
  marker.header.frame_id = "enu";
  marker.header.stamp = ros::Time::now();
  marker.id = 10000 + object.id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = object.pose;
  marker.scale = object.scale;
  marker.color.a = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
}

void marker_manager::get_text_marker(mil_msgs::PerceptionObject const& object, visualization_msgs::Marker& marker)
{
  marker.header.frame_id = "enu";
  marker.header.stamp = ros::Time::now();
  marker.id = 20000 + object.id;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = object.pose;
  marker.scale.z = 1;
  marker.text = object.classification + " | " + object.labeled_classification + " (" + std::to_string(object.id) + ")";
  marker.color.a = 0.7;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
}

visualization_msgs::InteractiveMarker marker_manager::get_marker_interactive(const mil_msgs::PerceptionObject& object)
{
  std::string id = std::to_string(object.id);
  std::string name = object.classification;
  visualization_msgs::InteractiveMarker int_marker;
  visualization_msgs::InteractiveMarkerControl control;
  int_marker.header.frame_id = "enu";
  int_marker.scale = 1;
  int_marker.name = name;
  int_marker.pose = object.pose;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  control.always_visible = true;
  control.name = name;
  int_marker.controls.push_back(control);
  return int_marker;
}

void marker_manager::update_markers(mil_msgs::PerceptionObjectArray const& objects)
{
  visualization_msgs::MarkerArray markers;

  visualization_msgs::Marker marker_delete;
  marker_delete.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.push_back(marker_delete);

  for (const auto& object : objects.objects)
  {
    visualization_msgs::Marker object_marker;
    visualization_msgs::Marker text_marker;
    get_object_marker(object, object_marker);
    get_text_marker(object, text_marker);
    markers.markers.push_back(object_marker);
    markers.markers.push_back(text_marker);
    // interactive_marker_server_->insert(get_marker_interactive(object.second));
    // menu_handler_.apply(*interactive_marker_server_, object.classification);
  }

  markers_pub_.publish(markers);
}

}  // pcodar namespace
