#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

class Dynamic_Move {
private:
	ros::Subscriber sub;
	ros::Publisher pub;
	double force;
	double maxForce;
	double move_rate;
	double old_secs = ros::Time::now().toSec();
	double secs;
public:
	Dynamic_Move(ros::NodeHandle *nh){
		force = 0;
		nh->getParam("/maxForce", maxForce);
		nh->getParam("/move_rate", move_rate); 
		old_secs = 0;
		secs = 0;
		pub = nh->advertise<geometry_msgs::WrenchStamped>("/wrench", 10);
		sub = nh->subscribe("/adaptive_controller_custom", 1000, &Dynamic_Move::move_callback, this);
	}
	
	void move_callback(const geometry_msgs::WrenchStamped::ConstPtr msg_){
			geometry_msgs::WrenchStamped msg;
			secs = ros::Time::now().toSec();
			
			if(force > maxForce){
				msg.wrench.force.x = 0;
				msg.wrench.force.y = msg_->wrench.force.y;
        msg.wrench.force.z = msg_->wrench.force.z;
        msg.wrench.torque.x = msg_->wrench.torque.x;
        msg.wrench.torque.y = msg_->wrench.torque.y;
        msg.wrench.torque.z = msg_->wrench.torque.z;
				pub.publish(msg);
			} else {
			
				if(secs > (old_secs + move_rate)){
					force += 1;
					ROS_INFO_STREAM("incremented");
					old_secs = secs;
				}
			
				msg.wrench.force.x = force;
				msg.wrench.force.y = msg_->wrench.force.y;
				msg.wrench.force.z = msg_->wrench.force.z;
				msg.wrench.torque.x = msg_->wrench.torque.x;
				msg.wrench.torque.y = msg_->wrench.torque.y;
				msg.wrench.torque.z = msg_->wrench.torque.z;
				pub.publish(msg);
			}
  }
};

int main (int argc, char **argv){
  ros::init(argc, argv, "dynamic_move");
  ros::NodeHandle nh;
	Dynamic_Move dm = Dynamic_Move(&nh);
  ros::spin();
}

