#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "angular_acc");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::WrenchStamped>("/wrench", 10);
  double torque;
  double maxTorque;
  double move_rate;
  nh.getParam("/move_rate", move_rate);
  nh.getParam("/maxTorque", maxTorque);
  double old_secs = ros::Time::now().toSec();
  double secs = 0;

  while (ros::ok())
  {
    geometry_msgs::WrenchStamped msg;
    secs = ros::Time::now().toSec();

    if (torque > maxTorque)
    {
      break;
    }
    if (secs > (old_secs + move_rate))
    {
      torque += 1;
      old_secs = secs;
    }

    msg.wrench.torque.z = torque;
    pub.publish(msg);
  }
}
