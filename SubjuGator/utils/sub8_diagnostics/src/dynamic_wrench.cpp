#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>


int main (int argc, char **argv){
	ros::init(argc, argv, "dynamic_test");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("/wrench", 10);
	double force;
	double secs = 0;
	double move_time;
	nh.getParam("/acceleration", force);
	nh.getParam("/move_time", move_time);
	ros::Rate rate(200);
	
	while(ros::ok()){
		geometry_msgs::WrenchStamped msg;
		secs = ros::Time::now().toSec();
		
		while (ros::Time::now().toSec() - move_time < secs){
			msg.wrench.torque.z = force;
			pub.publish(msg);
			rate.sleep();
		}
		
		force = -force;
		rate.sleep();
	}
}
