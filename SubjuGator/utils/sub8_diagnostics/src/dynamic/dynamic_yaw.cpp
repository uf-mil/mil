#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

class Dynamic_Yaw {
private:
	ros::Subscriber sub;
	ros::Publisher pub;
	double force;
	double maxForce;
	double move_rate;
	double old_secs = ros::Time::now().toSec();
	double secs;
public:
	Dynamic_Yaw(ros::NodeHandle *nh){
		force = 0;
		nh->getParam("/maxForce", maxForce);
		nh->getParam("/move_rate", move_rate); 
		old_secs = 0;
		secs = 0;
		pub = nh->advertise<geometry_msgs::WrenchStamped>("/wrench", 10);
		sub = nh->subscribe("/adaptive_controller_custom", 1000, &Dynamic_Yaw::yaw_callback, this);
	}
	
	void yaw_callback(const geometry_msgs::WrenchStamped::ConstPtr msg_){
			geometry_msgs::WrenchStamped msg;
			secs = ros::Time::now().toSec();
			
			if(force > maxForce){
				msg.wrench.force.x = msg_->wrench.force.x;
				msg.wrench.force.y = msg_->wrench.force.y;
        msg.wrench.force.z = msg_->wrench.force.z;
        msg.wrench.torque.x = msg_->wrench.torque.x;
        msg.wrench.torque.y = msg_->wrench.torque.y;
        msg.wrench.torque.z = 0;
				pub.publish(msg);
			} else {
			
				if(secs > (old_secs + move_rate)){
					force += 1;
					old_secs = secs;
				}
			
				msg.wrench.force.x = msg_->wrench.force.x;
				msg.wrench.force.y = msg_->wrench.force.y;
				msg.wrench.force.z = msg_->wrench.force.z;
				msg.wrench.torque.x = msg_->wrench.torque.x;
				msg.wrench.torque.y = msg_->wrench.torque.y;
				msg.wrench.torque.z = force;
				pub.publish(msg);
			}
  }
};

int main (int argc, char **argv){
  ros::init(argc, argv, "dynamic_yaw");
  ros::NodeHandle nh;
	Dynamic_Yaw dy = Dynamic_Yaw(&nh);
  ros::spin();
}

