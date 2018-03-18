#include <point_cloud_object_detection_and_recognition/marker_manager.hpp>

// #include <ros/console.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <tf/transform_datatypes.h>

namespace pcodar
{

void marker_manager::initialize(ros::NodeHandle& nh, id_label_map_ptr id_label_map)
{
    pub_markers_objects_ = nh.advertise<visualization_msgs::MarkerArray>("info_markers/objects", 10);
    pub_markers_text_ = nh.advertise<visualization_msgs::MarkerArray>("info_markers/text", 10);
    interactive_marker_server_ = std::unique_ptr<interactive_markers::InteractiveMarkerServer>(new interactive_markers::InteractiveMarkerServer("info_markers/interactive", "", false));
    id_label_map_ = id_label_map;
}
    void marker_manager::interative_marker_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){

    }
visualization_msgs::Marker marker_manager::get_marker_object(const mil_msgs::PerceptionObject& object)
{
    std::string id = std::to_string(object.id);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "enu";
    marker.header.stamp = ros::Time::now();
    marker.id = object.id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = object.pose;
    marker.scale = object.scale;
    // if (object.scale.x <= 0 || object.scale.y <= 0 || object.scale.z <= 0)
        // continue;
    // std::cout << "Object " << id << " Classification: " << object.classification << " pos x: " << object.pose.position.x << " Scale: " << marker.scale.x << " " << marker.scale.y << " " << marker.scale.z << std::endl;
    //
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    return marker;
}

visualization_msgs::Marker marker_manager::get_marker_text(const mil_msgs::PerceptionObject& object)
{
    std::string id = std::to_string(object.id);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "enu";
    marker.header.stamp = ros::Time::now();
    marker.id = object.id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = object.pose;
    marker.scale.z = 1;
    marker.text = object.classification + " | " + object.labeled_classification + " (" + id + ")";
        marker.color.a = 0.7;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
    return marker;
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

void marker_manager::update_markers(const id_object_map_ptr objects)
{
    visualization_msgs::MarkerArray marker_object_array;
    visualization_msgs::MarkerArray marker_text_array;

    visualization_msgs::Marker marker_delete;
    marker_delete.action = visualization_msgs::Marker::DELETEALL;
    // marker_array.markers.push_back(marker_delete);
    for (const auto& object : *objects)
    {
        auto it = id_label_map_->find(object.second.id);
        marker_object_array.markers.push_back(get_marker_object(object.second));
        marker_text_array.markers.push_back(get_marker_text(object.second));
        interactive_marker_server_->insert(get_marker_interactive(object.second));
        // menu_handler_.apply(*interactive_marker_server_, object.classification);
    }
    pub_markers_objects_.publish(marker_object_array);
    pub_markers_text_.publish(marker_text_array);
    // interactive_marker_server_->applyChanges();

}
}  // pcodar namespace
