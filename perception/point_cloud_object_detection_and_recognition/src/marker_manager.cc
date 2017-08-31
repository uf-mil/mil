#include <point_cloud_object_detection_and_recognition/marker_manager.hh>

// #include <ros/console.h>

#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <tf/transform_datatypes.h>

namespace pcodar
{

void marker_manager::initialize(ros::NodeHandle& nh)
{
    pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>("/nob/info_markers", 10);
}

void marker_manager::update_markers(const std::vector<mil_msgs::PerceptionObject>& objects)
{
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker_delete;
    marker_delete.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker_delete);

    for (const auto& object : objects)
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

        marker.color.a = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;

        marker_array.markers.push_back(marker);
    }

    pub_markers_.publish(marker_array);
}
}  // pcodar namespace