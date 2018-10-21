#include "navigator_marker.h"

const std::string frame_id = "/gps";
const std::string node_name = "navigator_marker";
const std::string pub_topic = "/satellite/navigator_marker";
const std::string sub_topic = "/satellite/navsatfix";

int main( int argc, char** argv )
{
  ros::init(argc, argv, "navigator_marker");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>(pub_topic, 1);

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;

    //setting the namespace
    marker.ns = node_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    //TODO customize the color
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    //TODO Do I want the marker to decay?
    marker.lifetime = ros::Duration();

    //publish the marker_pub

    //check if there are subscribers
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok()) return 0;
      ROS_WARN_ONCE("Please create a subscriber to " + pub_topic);
      sleep(1);
    }
    //publish
    marker_pub.publish(marker);

    //TODO what does this do?
    r.sleep();
  }
}
