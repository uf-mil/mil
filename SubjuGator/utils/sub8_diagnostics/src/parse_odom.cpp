#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <fstream>

class DataParser {

        private:
        ros::Subscriber sub;
	std::ofstream dataFile;
	std::string filepath;

        public:
        DataParser(ros::NodeHandle *nh) {
                //initialization of subcriber
		sub = nh->subscribe("/odom", 1000,
                        &DataParser::odomCallback, this);

		nh->getParam("/csv_path", filepath);

		//Setup file for data
   		dataFile.open(filepath + "odom.csv",
			       	std::ofstream::out | std::ofstream::trunc);
    		dataFile << "Time,PosX,PosY,PosZ,OriX,OriY,OriZ,OriW,LinX,LinY,LinZ,AngX,AngY,AngZ\n";
        }

        void odomCallback(const nav_msgs::Odometry::ConstPtr msg) {
		//Record Time
    		dataFile << msg->header.stamp.sec << "." << msg->header.stamp.nsec;
    		
		//Record Position
		dataFile << "," << msg->pose.pose.position.x;
		dataFile << "," << msg->pose.pose.position.y;
		dataFile << "," << msg->pose.pose.position.z;

		//Record Orientation (quaternion)
		dataFile << "," << msg->pose.pose.orientation.x;
		dataFile << "," << msg->pose.pose.orientation.y;
		dataFile << "," << msg->pose.pose.orientation.z;
		dataFile << "," << msg->pose.pose.orientation.w;

		//Record Linear Velocity
    		dataFile << "," << msg->twist.twist.linear.x;
		dataFile << "," << msg->twist.twist.linear.y;
		dataFile << "," << msg->twist.twist.linear.z;
		
		//Record Angular Velocity
		dataFile << "," << msg->twist.twist.angular.x;
		dataFile << "," << msg->twist.twist.angular.y;
		dataFile << "," << msg->twist.twist.angular.z << "\n";
        }

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "parse_odom_node");
  ros::NodeHandle nh;
  DataParser dp = DataParser(&nh);  
  ros::spin();

  return 0;
}

