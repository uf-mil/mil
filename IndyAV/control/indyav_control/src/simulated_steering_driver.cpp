#include <std_msgs/Float64.h>
#include <indyav_control/simulated_steering_driver.hpp>
#include <math.h>
#include <cmath>

template <class MSG>
SimulatedSteeringDriver<MSG>::SimulatedSteeringDriver(ros::NodeHandle* _nh, const std::string& sub_topic)
{
  nh_ = _nh;

  sub_ = nh_->subscribe(sub_topic, 1, &SimulatedSteeringDriver::Callback, this);

  // get all the controllers from the ros params
  std::string name = "/car";
  ROS_ASSERT_MSG(ros::param::has(name), "no car param namspace");
  name += "/simulated_hardware_controllers";
  ROS_ASSERT_MSG(nh_->hasParam(name), "no simualted hardware controllers param namspace");
  name += "/steering";
  ROS_ASSERT_MSG(nh_->hasParam(name), "no steering param namspace");

  auto left_name = name + "/left";
  ROS_ASSERT_MSG(ros::param::has(left_name), "no left controller");
  pubs_[left_name] = nh_->advertise<std_msgs::Float64>(left_name + "/command", 5);

  auto right_name = name + "/right";
  ROS_ASSERT_MSG(ros::param::has(right_name), "no right controller");
  pubs_[right_name] = nh_->advertise<std_msgs::Float64>(right_name + "/command", 5);
}

template <class MSG>
void SimulatedSteeringDriver<MSG>::Callback(const MSG& _msg)
{
  double cmd_angle = _msg.steering_angle;
  for (auto i = pubs_.begin(); i != pubs_.end(); ++i)
  {
        // TODO: impliment ackerman steering angles
    // Worked Mr. Daniel Williams and Mr. Jin-hyuk, Yu

    // Ackermann angle calculation
    double theta = cmd_angle; // an input angle based on geometric bicycle model for the vehicle
    double wf, wr, L, R1, A2, R2, theta_I, theta_O;
    wf = 94;// = the front wheel width
    wr = 96.5;// = the rear wheel width
    L =  104; // the length of the wheel base
    theta_I = tan^(-1)((2*L*sin(theta))/(2*L*cos(theta)-wf*sin(theta)));// Steering_angle_inside(left)
    theta_O = tan^(-1)((2*L*sin(theta))/(2*L*cos(theta)+wf*sin(theta));// Steering_angle_outside(right)
    // Boundary Condition 1 : Ackermann equation satisfy check
    R1 = (wf/2) + l/tan(theta_I);
    A2 = 56;// the distance from the rear wheel to the center of the kart
    R2 = sqrt(pow(A2,2)+pow(L,2)*pow((1/tan(theta)),2));
    if (R1 == R2)
    {
      R = R1;
    }
    // Boundary Condition 2 : Space requirement for turning 
    // Maximum and Minimum radius of vehicle turning
    double m1, m2, RMax, RMin;
    RMin = L/tan(theta) - wf/2 - wr/2;
    m1 = RMin + wr;
    m2 = L + g; // g is the length between the front wheel and bumper
    RMax = sqrt(pow(m1,2)+pow(m2,2));
    if (R>RMin && R<RMax)
    {
      theta_I = theta_L // the angle that will be put in the left side
      theta_O = theta_R // the angle that will be put in the right side
    }

    std_msgs::Float64 msg;
    msg.data = cmd_angle;
    i->second.publish(msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulated_steering_driver");
  ros::NodeHandle nh("~");
  std::string input_topic;
  ROS_ASSERT_MSG(nh.getParam("input_topic", input_topic), "No input topic specified");
  auto a = SimulatedSteeringDriver<indyav_control::SteeringStamped>(&nh, input_topic);
  ros::spin();
  return 0;
}
