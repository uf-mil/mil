#include <sub8_pcl/visualization.hpp>
#include <visualization_msgs/Marker.h>

namespace sub {

RvizVisualizer::RvizVisualizer(std::string rviz_topic) {
  rviz_pub = nh.advertise<visualization_msgs::Marker>(rviz_topic, 1);
  ros::spinOnce();
  // Give these guys some time to get ready
  ros::Duration(0.5).sleep();
}

void RvizVisualizer::visualize_buoy(geometry_msgs::Pose& pose, std::string& frame_id) {
  visualization_msgs::Marker marker;

  // Generate sphere marker for the buoy
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "sub";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 0.8;
  marker.color.r = 0.2;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  rviz_pub.publish(marker);

  // Generate text to overlay on the buoy (TODO: Put the text offset from the buoy)
  marker.header.stamp = ros::Time();
  marker.ns = "sub";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pose.position.x;
  // y is down in computer vision world
  marker.pose.position.y = pose.position.y - 0.25;
  marker.pose.position.z = pose.position.z;
  // Orientation doesn't matter for this guy
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.1;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.2;
  marker.color.b = 0.1;
  marker.text = "Buoy Bump Target";
  rviz_pub.publish(marker);
}

void RvizVisualizer::visualize_torpedo_board(geometry_msgs::Pose& pose, std::string& frame_id) {
  visualization_msgs::Marker marker;

  // Generate sphere marker for the buoy
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = "sub";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.a = 0.8;
  marker.color.r = 0.2;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  rviz_pub.publish(marker);

  // Generate text to overlay on the buoy (TODO: Put the text offset from the buoy)
  marker.header.stamp = ros::Time();
  marker.ns = "sub";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pose.position.x;
  // y is down in computer vision world
  marker.pose.position.y = pose.position.y - 0.25;
  marker.pose.position.z = pose.position.z;
  // Orientation doesn't matter for this guy
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.1;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.text = "Torpedo Board Centroid";
  rviz_pub.publish(marker);
}
}