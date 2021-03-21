#include <ros/ros.h>
#include <mil_msgs/MoveToActionGoal.h>


int main (int argc, char **argv){
  ros::init(argc, argv, "dynamic_move");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<mil_msgs::MoveToActionGoal>("/moveto/goal", 10);
  double acceleration;
  double move_time;
  double positionx;
  nh.getParam("/acceleration", acceleration);
  nh.getParam("/move_time", move_time);
  nh.getParam("/positionx", positionx);

  while(ros::ok()){
    mil_msgs::MoveToActionGoal msg;
		
   	//msg.goal.posetwist.pose.position.x = positionx;
		//msg.goal.posetwist.pose.orientation.x = 1;
		//msg.goal.posetwist.acceleration.linear.x = acceleration;
		msg.goal.posetwist.pose.orientation.z = 1;
    msg.goal.posetwist.pose.orientation.w = 0;
		msg.goal.posetwist.acceleration.angular.z = 15;
    pub.publish(msg);
		ros::Duration(move_time).sleep();

    positionx = -1*positionx;
		acceleration = -1*acceleration;
  }
}
