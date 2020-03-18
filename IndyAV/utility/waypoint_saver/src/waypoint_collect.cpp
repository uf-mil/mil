#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <fstream>
#include<math.h>
#include<iostream>
#include<string>
/**
 * This tutorial demonstrates simple receipt of position and speed of the Evarobot over the ROS system.
 */

/**
 * Callback function executes when new topic data comes.
 * Task of the callback function is to print data to screen.
 */
using namespace std;
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
double d=0;
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  std::cout<<"updating"<<std::endl;
  std::ofstream file;
  std::ifstream fs;
fs.open("waypoint.csv");
//read last line of file
if(fs.is_open())
  {
    fs.seekg(-1, std::ios_base::end);
    if(fs.peek() == '\n')
    {
      fs.seekg(-1, std::ios_base::cur);
      int i = fs.tellg();
      for(i;i > 0; i--)
      {
        if(fs.peek() == '\n')
        {
          //Found
          fs.get();
          break;
        }
        fs.seekg(i, std::ios_base::beg);
      }
    }
    std::string lastline;
    getline(fs, lastline);
cout<<"Last Line: "<<lastline<<endl;
//calculate distance of current odometry and from previous odometry
float a,b,c;
std::istringstream ss(lastline);
std::string token;
std::getline(ss, token, ',');
a=stod(token);
std::getline(ss, token, ',');
b=stod(token);
std::getline(ss, token, ',');
c=stod(token);  
d=sqrt(pow(a,2)+pow(b,2)+pow(c,2));  
}
double dx;
dx=sqrt(pow(msg->pose.pose.position.x,2)+pow(msg->pose.pose.position.y,2)+pow(msg->pose.pose.position.z,2))-d;
if (dx>0.5)
{
  file.open("waypoint.csv", std::ios::out | std::ios::app);
  file<<msg->pose.pose.position.x<<","<<msg->pose.pose.position.y<<","<<msg->pose.pose.position.z<<","<<msg->pose.pose.orientation.x<<","<<msg->pose.pose.orientation.y<<","<<msg->pose.pose.orientation.z<<","<<msg->pose.pose.orientation.w<<msg->twist.twist.linear.x<<"\n";
}
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "waypoint_collector");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);


  ros::spin();

  return 0;
}
