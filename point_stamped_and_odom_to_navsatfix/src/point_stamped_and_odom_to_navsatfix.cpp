#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>

void llaCallback(const geometry_msgs::PointStamped &msg);
void odomCallback(const nav_msgs::Odometry &msg);
sensor_msgs::NavSatFix* createNavSatFix();
void callback();

/*
* This executable listens for gps messages
* published by the NaviGator and publishes
* them as NavSatFix messages for the rviz_satellite
* package.
*/

const std::string gps_topic = "/satellite/navsatfix";
const std::string lla_topic = "/lla";
const std::string odometry_topic = "/odom";
const std::string frame_id = "/map";

const geometry_msgs::PointStamped* lla;
const nav_msgs::Odometry* odom;

ros::Publisher navSatFix_pub;

int main( int argc, char** argv)
{
  ros::init(argc, argv, "navigator2gps");
  ros::NodeHandle n;
  ros::Rate r(1);

  ros::Subscriber lla_sub = n.subscribe(lla_topic, 1, llaCallback);
  ros::Subscriber odom_sub = n.subscribe(odometry_topic, 1, odomCallback);
  navSatFix_pub = n.advertise<sensor_msgs::NavSatFix>(gps_topic, 1);
  ros::spin();
}

//Everytime a pointstamped msg or a odometry msg is heard, publish a navsatfix msg.
void llaCallback(const geometry_msgs::PointStamped &msg){
  lla = &msg;
  callback();
}
void odomCallback(const nav_msgs::Odometry &msg){
  odom = &msg;
  callback();
}

void callback(){
  sensor_msgs::NavSatFix* navsatfix = createNavSatFix();
  if (navsatfix == NULL) return;
  navSatFix_pub.publish(*navsatfix);
}



sensor_msgs::NavSatFix* createNavSatFix(){
  if (lla == NULL || odom == NULL) return NULL;

  sensor_msgs::NavSatFix* navsatfix = new sensor_msgs::NavSatFix();
  navsatfix->header.stamp = ros::Time::now();
  navsatfix->header.frame_id = frame_id;
  //gather from the vector3 message (where x is longitude, y is latitude, z is altitude)
  navsatfix->longitude = lla->point.y;
  navsatfix->latitude = lla->point.x;
  navsatfix->altitude = /*lla->point.z*/-39.0;
  /*
  double quaternion[4] = {
    odom->pose.pose.orientation.x,
    odom->pose.pose.orientation.y,
    odom->pose.pose.orientation.z,
    odom->pose.pose.orientation.w,
  };
  */
  return navsatfix;
}
