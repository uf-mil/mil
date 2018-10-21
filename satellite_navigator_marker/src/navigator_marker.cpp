#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/NavSatFix.h>

const sensor_msgs::NavSatFix* navsatfix;

void callback(const sensor_msgs::NavSatFix &msg){
  navsatfix = &msg;
}

const std::string frame_id = "/map";
const std::string node_name = "navigator_marker";
const std::string pub_topic = "/satellite/navigator_marker";
const std::string navsatfix_topic = "/satellite/navsatfix";

int main( int argc, char** argv )
{
  ros::init(argc, argv, node_name);
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>(pub_topic, 1);
  ros::Subscriber navsatfix_sub = n.subscribe(navsatfix_topic, 1, callback);
  while (ros::ok())
  {

    if (navsatfix == NULL) continue;

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = frame_id;

    marker.ns = node_name;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = navsatfix->latitude;
    marker.pose.position.y = navsatfix->longitude;
    marker.pose.position.z = -39;

    /* TODO: FIGURE OUT HOW TO CONVERT POS_COVARIANCE (float64[9]) TO QUATERNION FORM (x, y, z, w)
    marker.pose.orientation.x = navsatfix->position_covariance[0];
    marker.pose.orientation.y = navsatfix->position_covariance[4];
    marker.pose.orientation.z = navsatfix->position_covariance[8];
    marker.pose.orientation.w = 1.0f;
    */
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
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
      //ROS_WARN_ONCE("Please create a subscriber to " + pub_topic);
      sleep(1);
    }
    //publish
    marker_pub.publish(marker);

    //TODO what does this do?
    r.sleep();
  }
}
