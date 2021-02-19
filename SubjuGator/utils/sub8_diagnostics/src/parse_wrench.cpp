#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
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
		sub = nh->subscribe("/wrench", 1000,
                        &DataParser::wrenchCallback, this);

		nh->getParam("/csv_path", filepath);

		//Setup file for data
   		dataFile.open(filepath + "wrench.csv",
			       	std::ofstream::out | std::ofstream::trunc);
    		dataFile << "Time,ForceX,ForceY,ForceZ,TorqueX,TorqueY,TorqueZ\n";
        }

        void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr msg) {
		//Record Time
    		dataFile << msg->header.stamp.sec << "." << msg->header.stamp.nsec;
    		
		//Record Force
		dataFile << "," << msg->wrench.force.x;
		dataFile << "," << msg->wrench.force.y;
		dataFile << "," << msg->wrench.force.z;

		//Record Torque
		dataFile << "," << msg->wrench.torque.x;
		dataFile << "," << msg->wrench.torque.y;
		dataFile << "," << msg->wrench.torque.z << "\n";
        }

};

int main(int argc, char **argv) {

  ros::init(argc, argv, "parse_wrench_node");
  ros::NodeHandle nh;
  DataParser dp = DataParser(&nh);  
  ros::spin();

  return 0;
}

