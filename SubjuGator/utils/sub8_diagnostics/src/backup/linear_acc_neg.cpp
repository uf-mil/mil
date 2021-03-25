#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

int main (int argc, char **argv){
	ros::init(argc, argv, "linear_acc");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("/wrench", 10);
	double force = 0;
	double maxForce;
	double move_rate;
	nh.getParam("/move_rate", move_rate);
	nh.getParam("/maxForce", maxForce);
	double old_secs = ros::Time::now().toSec();
	double secs = 0;

	while(ros::ok()){
		geometry_msgs::WrenchStamped msg;
		secs = ros::Time::now().toSec();
		
		if(force < maxForce){
			break;
		}
		if(secs > (old_secs + move_rate)){
			force -= 1.0;
			old_secs = secs;
		}
		
		msg.wrench.force.x = force;
		pub.publish(msg);
	}
}
