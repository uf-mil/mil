#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/NavSatFix.h>

void llaCallback(const geometry_msgs::Vector3 &msg);
void odomCallback(const nav_msgs::Odometry &msg);
sensor_msgs::NavSatFix createNavSatFix();
