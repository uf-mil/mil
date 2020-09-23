#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
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
  ros::Subscriber joy;
  ros::Publisher steering_pub_, throttle_pub_, brake_pub_;
};


JoyTeleop::JoyTeleop():
  steer_(0),
  throttle_(5),
  brake_(2)
{
  nh_.param("axis_steer", steer_, steer_);
  nh_.param("axis_throttle", throttle_, throttle_);
  nh_.param("axis_brake", brake_, brake_);
  nh_.param("scale_steer", s_scale, s_scale);
  nh_.param("scale_trigger", t_scale, t_scale);
  
  throttle_pub_ = nh_.advertise<indyav_control::ThrustStamped>("throttle", 1);
  brake_pub_ = nh_.advertise<indyav_control::ThrustStamped>("throttle",1);
  steering_pub_ = nh_.advertise<indyav_control::SteeringStamped>("steering", 1);
  joy = nh_.subscribe<sensor_msgs::Joy>("joy",100, &JoyTeleop::Callback, this);
}

void JoyTeleop::Callback(const sensor_msgs::Joy::ConstPtr& joy)
{
  indyav_control::ThrustStamped thrust;
  indyav_control::SteeringStamped steering_angle;
 
  steering_angle.steering_angle = s_scale*joy->axes[steer_];
 
  if (joy->axes[brake_] < 0.8) 
  {
    thrust.thrust = -1 * (t_scale*( abs (joy->axes[brake_] - 1) ) );
  }
  else
  { 
    thrust.thrust = t_scale*(abs (joy->axes[throttle_] - 1) );
  }
  steering_pub_.publish(steering_angle);
  throttle_pub_.publish(thrust);
  brake_pub_.publish(thrust);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  JoyTeleop joy_teleop;
	
  ros::spin();

}














































