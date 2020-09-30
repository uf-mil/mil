#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <string>
#include <geometry_msgs/Twist.h>
#include <indyav_control/ThrustStamped.h>
#include <indyav_control/SteeringStamped.h>
#include <sstream>


class JoyTeleop
{
public:
	JoyTeleop();
private:
void Callback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int steer_, throttle_, brake_;
  double s_scale, t_scale;
  std::string t_advertise, s_advertise;
  ros::Subscriber joy;
  ros::Publisher steering_pub_, throttle_pub_, brake_pub_;
};


JoyTeleop::JoyTeleop()
{
  nh_.getParam("axis_steer", steer_);
  nh_.getParam("axis_throttle", throttle_);
  nh_.getParam("axis_brake", brake_);
  nh_.getParam("scale_steer", s_scale);
  nh_.getParam("scale_trigger", t_scale);
  nh_.getParam("throttle_topic", t_advertise);
  nh_.getParam("steering_topic", s_advertise);

  if (nh_.hasParam("axis_steer"))
  {
  std::cout << "axis_steer\n";
  }else{
  std::cout << "no axis_steer\n";
  }  
  if (nh_.hasParam("axis_throttle"))
  {
  std::cout << "axis_throttle\n";
  }else{
  std::cout << "no axis_throttle\n";
  }
  if (nh_.hasParam("axis_brake"))
  {
  std::cout << "axis_brake\n";
  }else{
  std::cout <<"no axis_brake\n";
  }
  if (nh_.hasParam("scale_steer"))
  {
  std::cout << "scale_steer\n";
  }else{
  std::cout << "no scale_steer\n";
  }
  if (nh_.hasParam("scale_trigger"))
  {
  std::cout << "scale_trigger\n";
  }else{
  std::cout << "no scale_trigger\n";
  }
  if (nh_.hasParam("steering_topic"))
  {
  std::cout << "steering_topic\n";
  }else{
  std::cout <<"no steering_topic\n";
  }
  if (nh_.hasParam("throttle_topic"))
  {
  std::cout << "throttle_topic\n";
  }else{
  std::cout << "no throttle_topic\n";
  }

  ROS_INFO("%s", t_advertise);
  throttle_pub_ = nh_.advertise<indyav_control::ThrustStamped>(t_advertise, 1);
  steering_pub_ = nh_.advertise<indyav_control::SteeringStamped>(s_advertise, 1);
  joy = nh_.subscribe<sensor_msgs::Joy>("joy",100, &JoyTeleop::Callback, this);
}

void JoyTeleop::Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  indyav_control::ThrustStamped thrust;
  indyav_control::SteeringStamped steering_angle;
 
  steering_angle.steering_angle = s_scale*joy->axes.at(steer_);
 
  if (joy->axes[brake_] < 0.8) 
  {
    thrust.thrust = -1 * (t_scale*( abs (joy->axes.at(brake_) - 1) ) );
  }
  else
  { 
    thrust.thrust = t_scale*(abs (joy->axes.at(throttle_) - 1) );
  }
  steering_pub_.publish(steering_angle);
  throttle_pub_.publish(thrust);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  JoyTeleop joy_teleop;
	
  ros::spin();

}
