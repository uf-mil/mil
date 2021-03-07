#include <ros/ros.h>
#include <mil_msgs/MoveToActionGoal.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <cmath>

bool switch_ = true;
double w = 1;
double z = 0;
double acceleration = 15;

void callback(const nav_msgs::Odometry& msg){
 	//ROS_INFO_STREAM("w is " << w);
  //ROS_INFO_STREAM("z is " << z);
  //ROS_INFO_STREAM("posew is " << msg.pose.pose.orientation.w);
  //ROS_INFO_STREAM("posez is " << msg.pose.pose.orientation.z);
	if(round(w) == 1 && std::abs(msg.pose.pose.orientation.w) > 0.95){
		switch_ = true;
		w = 0;
		z = 1;
    ROS_INFO("w is true");
	} else if (round(z) == 1 && std::abs(msg.pose.pose.orientation.z) > 0.95){
		switch_ = true;
		z = 0;
		w = 1;
    ROS_INFO("z is true");
	}
}

int main (int argc, char **argv){
  ros::init(argc, argv, "dynamic_yaw");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<mil_msgs::MoveToActionGoal>("/moveto/goal", 10);
	ROS_INFO("Initialized");

	ros::Subscriber sub = nh.subscribe("/odom", 1000, callback);
  double move_time = 5;

	while(ros::ok()){
  	if(switch_ == true){
			ROS_INFO_STREAM("acceleration is " << acceleration);
			mil_msgs::MoveToActionGoal msg;
    	msg.goal.posetwist.pose.orientation.z = 1;
			msg.goal.posetwist.pose.orientation.w = 0;
    	msg.goal.posetwist.acceleration.angular.z = acceleration;
			pub.publish(msg);
			acceleration = -1*acceleration;
			switch_ = false;
		}
		ros::spinOnce();
	}
}

