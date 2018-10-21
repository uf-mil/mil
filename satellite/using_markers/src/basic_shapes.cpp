#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>


void updateLatLong(const sensor_msgs::NavSatFix &msg);
void updateOrientation(const nav_msgs::Odometry &msg);

double longitude = 29.5348776124;
double latitude = -82.3037160738;
double altitude = -195.996072833;

double east = 0;
double north = 0;
double up = 0;
double roll = 0;

const std::string navsatfix_topic = "/satellite/navsatfix";
const std::string odom_topic = "/odom";


int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber lla_sub = n.subscribe(navsatfix_topic, 1, updateLatLong);
  ros::Subscriber odom_sub = n.subscribe(odom_topic, 1, updateOrientation);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    const double rho = M_PI / 180;
    const double lat_rad = latitude * rho;

    unsigned int n = /*(1 << zoom)*/ 1;
    marker.pose.position.x = n * ((longitude + 180) / 360.0);
    marker.pose.position.y = n * (1 - (std::log(std::tan(lat_rad) + 1 / std::cos(lat_rad)) / M_PI)) /
        2;
    marker.pose.position.z = 0;

    marker.pose.orientation.x = east;
    marker.pose.orientation.y = north;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = roll;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 10.0;
    marker.scale.y = 5.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }

    r.sleep();
  }
}

void updateLatLong(const sensor_msgs::NavSatFix &msg){
  longitude = msg.longitude;
  latitude = msg.latitude;
  altitude = msg.altitude;
}

void updateOrientation(const nav_msgs::Odometry &msg){
  east = msg.pose.pose.orientation.x;
  north = msg.pose.pose.orientation.y;
  up = msg.pose.pose.orientation.z;
  roll = msg.pose.pose.orientation.w;

}
