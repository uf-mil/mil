#include <point_cloud_object_detection_and_recognition/marker_manager.hpp>

// #include <ros/console.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <tf/transform_datatypes.h>

namespace pcodar
{
void MarkerManager::initialize(ros::NodeHandle& nh)
{
  markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("objects_visualization", 10);
}

void MarkerManager::get_object_marker(mil_msgs::PerceptionObject const& object, visualization_msgs::Marker& marker)
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

void MarkerManager::get_text_marker(mil_msgs::PerceptionObject const& object, visualization_msgs::Marker& marker)
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

void MarkerManager::update_markers(mil_msgs::PerceptionObjectArray const& objects)
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
  }

  markers_pub_.publish(markers);
}

}  // pcodar namespace
