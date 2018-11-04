#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navigator_marker");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("navigator_marker", 1);

  //TODO choose a customized shape for the marker
  uint32_t shape = visualization_msgs::Marker::ARROW;

  while (ros::ok())
  {
    //instantiate the marker. This arrow represents the navigator.
    visualization_msgs::Marker marker;

    //set the header. Current frame_id = "/my_frame". Idk what this does.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    //set the namespace to "navigator_marker"
    marker.ns = "navigator_marker";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;


    //TODO update position and orientation with NAVIGATOR data
    int position[3] = {0, 0, 0};
    int orientation[3] = {0.0, 0.0, 0.0};

    marker.pose.position.x = position[0];
    marker.pose.position.y = position[1];
    marker.pose.position.z = position[2];

    marker.pose.orientation.x = orientation[0];
    marker.pose.orientation.y = orientation[1];
    marker.pose.orientation.z = orientation[2];
    marker.pose.orientation.w = 1.0;

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
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    //publish
    marker_pub.publish(marker);

    //TODO what does this do?
    r.sleep();
  }
}
