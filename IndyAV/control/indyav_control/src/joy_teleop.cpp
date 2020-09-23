#include <geometry_msgs/Twist.h>
#include <indyav_control/RevsStamped.h>
#include <indyav_control/SteeringStamped.h>

#include <sstream>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

class JoyTeleop
{
public:
  JoyTeleop(ros::NodeHandle* _nh);

private:
  void Callback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle* nh_;

  int steer_axis_, throttle_axis_;
  double s_scale_, r_scale_;
  std::string t_advertise_, s_advertise_;
  ros::Subscriber joy_;
  ros::Publisher steering_pub_, throttle_pub_;
};

JoyTeleop::JoyTeleop(ros::NodeHandle* _nh)
{
  nh_ = _nh;

  if (nh_->getParam("axis_steer", steer_axis_))
  {
    ROS_INFO("Got param 'axis_steer': %d", steer_axis_);
  }
  else
  {
    ROS_ERROR("Failed to get param 'axis_steer'");
  }
  if (nh_->getParam("axis_throttle", throttle_axis_))
  {
    ROS_INFO("Got param 'axis_throttle': %d", throttle_axis_);
  }
  else
  {
    ROS_ERROR("Failed to get param 'axis_throttle'");
  }
  if (nh_->getParam("scale_steer", s_scale_))
  {
    ROS_INFO("Got param 'scale_steer': %f", s_scale_);
  }
  else
  {
    ROS_ERROR("Failed to get param 'scale_steer'");
  }
  if (nh_->getParam("scale_trigger", r_scale_))
  {
    ROS_INFO("Got param 'scale_trigger': %f", r_scale_);
  }
  else
  {
    ROS_ERROR("Failed to get param 'scale_trigger'");
  }
  if (nh_->getParam("throttle_topic", t_advertise_))
  {
    ROS_INFO("Got param 'throttle_topic': %s", t_advertise_.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get param 'throttle_topic'");
  }
  if (nh_->getParam("steering_topic", s_advertise_))
  {
    ROS_INFO("Got param 'steering_topic': %s", s_advertise_.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get param 'steering_topic'");
  }

  throttle_pub_ = nh_->advertise<indyav_control::RevsStamped>(t_advertise_, 1);
  steering_pub_ = nh_->advertise<indyav_control::SteeringStamped>(s_advertise_, 1);
  joy_ = nh_->subscribe<sensor_msgs::Joy>("/joy", 100, &JoyTeleop::Callback, this);
}

void JoyTeleop::Callback(const sensor_msgs::Joy::ConstPtr& _joy)
{
  static indyav_control::RevsStamped radians_per_second;
  static indyav_control::SteeringStamped steering_angle;

  steering_angle.steering_angle = s_scale_ * _joy->axes.at(steer_axis_);

  // Gamepad trigger incorrectly defaults to 0 when resting position is 1. Moving the trigger updates the value,
  // correcting the error. To seamlessly work around the problem, the angular velocity is not assigned until this change
  // is detected.
  static bool gamepad_init = false;
  if (_joy->axes.at(throttle_axis_) != 0)
    gamepad_init = true;
  if (gamepad_init)
    radians_per_second.radians_per_second = r_scale_ * (abs(_joy->axes.at(throttle_axis_) - 1)) / 2;

  steering_pub_.publish(steering_angle);
  throttle_pub_.publish(radians_per_second);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_teleop");
  ros::NodeHandle nh("~");
  JoyTeleop joy_teleop(&nh);
  ros::spin();
}