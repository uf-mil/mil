// %Tag(FULL)%
// %Tag(INCLUDE)%
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
// %EndTag(INCLUDE)%

// %Tag(CLASSDEF)%
class JoystickWrench
{
   public:
     JoystickWrench();

   private:
     void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
     ros::NodeHandle nh_;
     double force_scale_;
     double torque_scale_;
     int axis_force_;
     int axis_torque_;
     ros::Publisher wrench_pub_;
     ros::Subscriber joy_sub_;
     ros::Publisher vel_pub_;

};
// %EndTag(CLASSDEF)%

JoystickWrench::JoystickWrench()
{
// %Tag(Param)%
   nh_.param("force_scale", force_scale_, force_scale_);
   nh_.param("torque_scale", torque_scale_, torque_scale_);
   nh_.param("axis_force", axis_force_, axis_force_);
   nh_.param("axis_torque", axis_torque_, axis_torque_);
// %EndTag(PARAMS)% 

// %Tag(PUB)%
   wrench_pub_=nh_.advertise<geometry_msgs::Wrench>("motor/cmd_wrench", 1);
// %EndTag(PUB)%

// %Tag(Wrench to Turtle)%
   vel_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
// %EndTag(Wrench to Turtle)%

// %Tag(SUB)%
   joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoystickWrench::joyCallback, this);
// %EndTag(SUB)%

}

// %Tag(CALLBACK)%
void JoystickWrench::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  //Motor::Wrench wrench
  geometry_msgs::Wrench wrench;
  wrench.force.x = force_scale_*joy->axes[axis_force_];
  wrench.torque.z = torque_scale_*joy->axes[axis_torque_];
  wrench_pub_.publish(wrench);

  //turtlesim::Velocity vel;
  geometry_msgs::Twist vel;
  vel.angular.z = torque_scale_*joy->axes[axis_torque_];
  vel.linear.x = force_scale_*joy->axes[axis_force_];
  vel_pub_.publish(vel);

}
// %EndTag(CALLBACK)%

// %Tag(MAIN)%
int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_wrench");
  JoystickWrench joystick_wrench;
  
  ros::spin();
}
// %EndTag(MAIN)%
// %EndTag(FULL)%

