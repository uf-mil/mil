#include <sub8_vision_lib/visualization.hpp>
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
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration();
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
  marker.scale.z = 0.2;

  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration();
  marker.text = "Buoy Bump Target";
  rviz_pub.publish(marker);
}

void RvizVisualizer::visualize_torpedo_board(geometry_msgs::Pose& pose, Eigen::Quaterniond orientation,
                                             std::vector<Eigen::Vector3d>& targets, std::vector<Eigen::Vector3d>& corners3d, 
                                             std::string& frame_id) {
  visualization_msgs::Marker centroid, target_markers, text, borders;
  centroid.header.frame_id = target_markers.header.frame_id = text.header.frame_id = borders.header.frame_id = frame_id;
  centroid.header.stamp = target_markers.header.stamp = text.header.stamp = borders.header.stamp = ros::Time::now();
  centroid.ns = target_markers.ns = text.ns = borders.ns = "torpedo_board";
  centroid.lifetime = target_markers.lifetime = text.lifetime = borders.lifetime = ros::Duration();

  // Generate sphere marker for the torpedo_board centroid
  centroid.id = 0;
  centroid.type = visualization_msgs::Marker::SPHERE;
  centroid.action = visualization_msgs::Marker::ADD;
  centroid.pose = pose;
  centroid.scale.x = 0.1;
  centroid.scale.y = 0.1;
  centroid.scale.z = 0.1;
  centroid.color.a = 1.0;
  centroid.color.r = 0.0;
  centroid.color.g = 1.0;
  centroid.color.b = 0.0;
  // centroid.pose.orientation.x = orientation.x();
  // centroid.pose.orientation.y = orientation.y();
  // centroid.pose.orientation.z = orientation.z();
  // centroid.pose.orientation.w = orientation.w();
  rviz_pub.publish(centroid);

  // Generate markers denoting the centers of individual targets
  target_markers.id = 1;
  target_markers.type = visualization_msgs::Marker::SPHERE_LIST;
  target_markers.action = visualization_msgs::Marker::ADD;
  geometry_msgs::Point p;
  for(size_t i = 0; i < targets.size(); i++){
    p.x = targets[i](0);
    p.y = targets[i](1);
    p.z = targets[i](2);
    target_markers.points.push_back(p);
  }
  target_markers.scale.x = 0.1;
  target_markers.scale.y = 0.1;
  target_markers.scale.z = 0.1;
  target_markers.color.a = 1.0;
  target_markers.color.r = 0.0;
  target_markers.color.g = 1.0;
  target_markers.color.b = 0.0;
  // rviz_pub.publish(target_markers);

  // Generate text to overlay on the torpedo_board centroid
  text.id = 2;
  text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::Marker::ADD;
  text.pose.position.x = pose.position.x;
  text.pose.position.y = pose.position.y - 0.25;
  text.pose.position.z = pose.position.z;
  // Orientation doesn't matter for this guy
  text.scale.x = 0.1;
  text.scale.y = 0.1;
  text.scale.z = 0.1;
  text.color.a = 1.0;
  text.color.r = 1.0;
  text.color.g = 1.0;
  text.color.b = 0.0;
  text.text = "Torpedo Board Centroid";
  rviz_pub.publish(text);

  // Generate border line markers
  borders.id = 3;
  borders.type = visualization_msgs::Marker::LINE_STRIP;
  borders.action = visualization_msgs::Marker::ADD;
  borders.pose.orientation.w = 1.0;
  for(size_t i = 0; i <= corners3d.size(); i++){
    if(i == corners3d.size()){
      p.x = corners3d[0](0);
      p.y = corners3d[0](1);
      p.z = corners3d[0](2);
    }
    else{
      p.x = corners3d[i](0);
      p.y = corners3d[i](1);
      p.z = corners3d[i](2);
    } 
    borders.points.push_back(p);
  }
  borders.scale.x = 0.05;
  borders.scale.y = 0.05;
  borders.scale.z = 0.05;
  borders.color.a = 1.0;
  borders.color.r = 1.0;
  borders.color.g = 1.0;
  borders.color.b = 0.0;
  rviz_pub.publish(borders);
}

} // End namespace "sub"