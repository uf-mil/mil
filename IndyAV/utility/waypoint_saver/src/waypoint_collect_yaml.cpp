#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <fstream>
#include<math.h>
#include<iostream>
#include<string>
#include<yaml-cpp/yaml.h>
#include<vector>
/**
 * This tutorial demonstrates simple receipt of position and speed of the Evarobot over the ROS system.
 */

/**
 * Callback function executes when new topic data comes.
 * Task of the callback function is to print data to screen.
 */

//void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
//{
//vector<double>previous;
//vector<vector<double>> collected_odometry;
//double d=0;
//  std::cout<<"updating"<<std::endl;
//  std::ofstream file;
//  std::ifstream fs;
//fs.open("waypoint.yaml");
//read last line of file
//if (fs.is_open())
//{fs.close();
//YAML::Node waypoints = YAML::LoadFile("waypoints.yaml");
//collected_odometry=waypoints["odom_vector"].as<std::vector<std::vector<double>>();
//int n=collected_odometry.size();
//previous=collected_odometry[n-1];
//d=sqrt(pow(previous[0],2)+pow(previous[1],2)+pow(previous[2],2));
//}
//double dx;
//dx=sqrt(pow(msg->pose.pose.position.x,2)+pow(msg->pose.pose.position.y,2)+pow(msg-//>pose.pose.position.z,2))-d;
//if (dx>0.5)
//{
//vector<double> vector_update;
//vector_update.push_back(msg->pose.pose.position.x);
//vector_update.push_back(msg->pose.pose.position.y);
//vector_update.push_back(msg->pose.pose.position.z);
//collected_odometry.push_back(vector_update);
//YAML::Emitter out;
//out << YAML::BeginSeq;
//out << YAML::Flow << collected_odometry;

//std::ofstream fout("waypoint.yaml");
  //fout<<out<<"\n";
//}
//}

using namespace std;
class waypoint_publisher
{
private:
ros::Publisher pub;
ros::Subscriber sub;
ros::NodeHandle n;
public:
vector<vector<double>> collected_waypoints;
vector<double>previous;
waypoint_publisher()
{
sub = n.subscribe("odom", 1000, &waypoint_publisher::Callback,this);
}
void Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
double d=0;
double dx=0;

cout<<collected_waypoints.size()<<endl;
std::ofstream file;
if(!collected_waypoints.empty())
{
	previous=collected_waypoints[collected_waypoints.size()-1];
	d=sqrt(pow(previous[0],2)+pow(previous[1],2)+pow(previous[2],2));
}
dx=sqrt(pow(msg->pose.pose.position.x,2)+pow(msg->pose.pose.position.y,2)+pow(msg->pose.pose.position.z,2))-d;
cout<<dx<<endl;
if (dx>0.5)
{
	vector<double> position;
	position.push_back(msg->pose.pose.position.x);
	position.push_back(msg->pose.pose.position.y);	
	position.push_back(msg->pose.pose.position.z);
	position.push_back(msg->pose.pose.orientation.x);
	position.push_back(msg->pose.pose.orientation.y);
	position.push_back(msg->pose.pose.orientation.z);
	position.push_back(msg->pose.pose.orientation.w);
	position.push_back(msg->twist.twist.linear.x);	
	collected_waypoints.push_back(position);
	cout<<collected_waypoints.size();

	std::ofstream fout("config.yaml");	
	YAML::Node node;  // starts out as null
	node["key"] = "waypoint";  // it now is a map nod
	node["seq"].push_back(collected_waypoints);
	fout<<node;
}

}

};




int main(int argc, char **argv)
{

  ros::init(argc, argv, "waypoint_collector_yaml");

  waypoint_publisher p;
  

  cout<<p.collected_waypoints.size()<<endl;
  ros::spin();

  return 0;
}
