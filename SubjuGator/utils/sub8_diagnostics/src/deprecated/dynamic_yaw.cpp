#include <ros/ros.h>
#include <mil_msgs/MoveToActionGoal.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <cmath>

class Dynamic_Yaw {
private:
	ros::Subscriber sub;
	ros::Publisher pub;
	int acceleration;
	int z;
	int w;
	bool switch_;
public:
	Dynamic_Yaw(ros::NodeHandle *nh){
		pub = nh->advertise<mil_msgs::MoveToActionGoal>("/moveto/goal", 10);
		sub = nh->subscribe("/odom", 1000, &Dynamic_Yaw::yaw_callback, this);
		z = 1;
		w = 0;
		acceleration = 10;
		switch_ = true;
		ROS_INFO_STREAM("initialized");
	}
	
	void yaw_callback(const nav_msgs::Odometry::ConstPtr msg){
  	if(round(w) == 1 && std::abs(msg->pose.pose.orientation.w) > 0.90){
    	switch_ = true;
    	w = 0;
    	z = 1;
  	} else if (round(z) == 1 && std::abs(msg->pose.pose.orientation.z) > 0.90){
    	switch_ = true;
    	z = 0;
    	w = 1;
  	}
		
		if(switch_ == true){
      mil_msgs::MoveToActionGoal msg;

      msg.goal.posetwist.pose.orientation.z = z;
      msg.goal.posetwist.pose.orientation.w = w;
      msg.goal.posetwist.acceleration.angular.z = acceleration;
      pub.publish(msg);
      switch_ = false;
    }

	}

};

int main (int argc, char **argv){
  ros::init(argc, argv, "dynamic_yaw");
  ros::NodeHandle nh;
	Dynamic_Yaw dy = Dynamic_Yaw(&nh);
  ros::spin();
}

