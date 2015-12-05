// %Tag(FULL)%
// %Tag(INCLUDE)%
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "navigator_msg_multiplexer/wrench_arbiter.h"
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
     int axis_x_force_;
     int axis_y_force_;
     int axis_torque_;
     ros::Publisher wrench_pub_;
     ros::Subscriber joy_sub_;
     ros::Publisher vel_pub_;
     ros::ServiceClient client;
     bool wrench_controller;
     int last_controller_state;
     int last_kill_state;
     int killed;

};
// %EndTag(CLASSDEF)%

JoystickWrench::JoystickWrench()
{
// %Tag(Param)%
   nh_.param("force_scale", force_scale_, force_scale_);
   nh_.param("torque_scale", torque_scale_, torque_scale_);
// %EndTag(PARAMS)% 

// %Tag(PUB)%
   wrench_pub_=nh_.advertise<geometry_msgs::WrenchStamped>("/wrench/rc", 1);
   client = nh_.serviceClient<navigator_msg_multiplexer::wrench_arbiter>("change_wrench");
   wrench_controller = false;
   killed = false;
   last_controller_state = 0;
   last_kill_state = 0;
// %EndTag(PUB)%

// %Tag(SUB)%
   joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoystickWrench::joyCallback, this);
// %EndTag(SUB)%

}

// %Tag(CALLBACK)%
void JoystickWrench::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{


  //Motor::Wrench wrench
  geometry_msgs::WrenchStamped wrench;
  navigator_msg_multiplexer::wrench_arbiter w_control;
  
  
  if (joy->buttons[7] == 1 && joy->buttons[7] != last_controller_state)
  {
    wrench_controller = !(wrench_controller);
    if (wrench_controller == false)
    {
      w_control.request.str = "rc";
      if (!client.call(w_control))
      {
        ROS_ERROR("Failed to change controller");
      }
    }
    if (wrench_controller == true)
    {
      w_control.request.str = "autonomous";
      if (!client.call(w_control))
      {
        ROS_ERROR("Failed to change controller");
      }
    }
  }

  if (killed == false)
  {
    wrench.wrench.force.x = force_scale_*joy->axes[1];
    wrench.wrench.force.y = -1*force_scale_*joy->axes[0];
    wrench.wrench.torque.z = -1*torque_scale_*joy->axes[3];
  }

  if (joy->buttons[8] == 1 && joy->buttons[8] != last_controller_state)
  {
    killed = !(killed);
  }
  last_controller_state = joy->buttons[7];
  last_kill_state = joy->buttons[8];
  wrench_pub_.publish(wrench);

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

