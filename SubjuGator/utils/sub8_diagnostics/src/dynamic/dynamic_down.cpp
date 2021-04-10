#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

class Dynamic_Down
{
private:
  ros::Subscriber sub;
  ros::Publisher pub;
  double force;
  double maxForce;
  double move_rate;
  double old_secs = ros::Time::now().toSec();
  double secs;

public:
  Dynamic_Down(ros::NodeHandle *nh)
  {
    force = 0;
    nh->getParam("/maxForce", maxForce);
    nh->getParam("/move_rate", move_rate);
    old_secs = 0;
    secs = 0;
    pub = nh->advertise<geometry_msgs::WrenchStamped>("/wrench", 10);
    sub = nh->subscribe("/adaptive_controller_custom", 1000, &Dynamic_Down::move_callback, this);
  }

  void move_callback(const geometry_msgs::WrenchStamped::ConstPtr msg_)
  {
    geometry_msgs::WrenchStamped msg;
    secs = ros::Time::now().toSec();

    if (secs - move_rate > old_secs)
    {
			maxForce = -maxForce;
			old_secs = secs;
    }
    else
    {
      msg.header.stamp = ros::Time::now();
      msg.wrench.force.x = msg_->wrench.force.x;
      msg.wrench.force.y = msg_->wrench.force.y;
      msg.wrench.force.z = maxForce;
      msg.wrench.torque.x = msg_->wrench.torque.x;
      msg.wrench.torque.y = msg_->wrench.torque.y;
      msg.wrench.torque.z = msg_->wrench.torque.z;
      pub.publish(msg);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_down");
  ros::NodeHandle nh;
  Dynamic_Down dm = Dynamic_Down(&nh);
  ros::spin();
}
