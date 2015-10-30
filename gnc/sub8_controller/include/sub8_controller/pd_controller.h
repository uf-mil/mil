#include <eigen_conversions/eigen_msg.h>
#include <dynamic_reconfigure/server.h>
#include <deque>
#include "sub8_controller/GainConfig.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/WrenchStamped.h"
#include "sub8_msgs/Trajectory.h"

typedef std::deque<Eigen::Vector3d> controller_deque;

class PDController {
  /*
    [1] One day, all the land will be boxplus
        http://arxiv.org/pdf/1107.1119.pdf
    TODO: Make this an abstract class
  */
public:
  PDController();

private:
  // ROS Miscellanea
  ros::NodeHandle nh;
  ros::Subscriber waypoint_sub;
  ros::Subscriber truth_sub;
  ros::Publisher wrench_pub;
  ros::Timer control_timer;
  dynamic_reconfigure::Server<sub8_controller::GainConfig> server;
  dynamic_reconfigure::Server<sub8_controller::GainConfig>::CallbackType reconfigure_callback;

  bool ready = false; // Wait until we're ready
  bool got_state = false;
  bool got_target = false;
  double control_period = 0.02;

  // Pose-trackings
  struct PoseStruct {
    Eigen::Vector3d position;
    Eigen::Matrix3d orientation;
    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;
  };

  PoseStruct target_state;
  PoseStruct current_state;

  // Control Parameters
  unsigned int trans_history_length = 75;
  unsigned int angle_history_length = 75;
  controller_deque translation_error_history;
  controller_deque orientation_error_history;

  // TODO: Make these ROSParams
  double kp_trans = 30;
  double kd_trans = 13;
  double ki_trans = 25;

  double kp_angle = 29;
  double kd_angle = 14;
  double ki_angle = 5;
  double sub_mass = 25; // kg

  // Callbacks
  void gain_callback(sub8_controller::GainConfig &config, uint32_t level);

  void waypoint_callback(const sub8_msgs::Trajectory::ConstPtr &);
  void truth_callback(const nav_msgs::Odometry::ConstPtr &);
  geometry_msgs::WrenchStamped msg_from_wrench(Eigen::Vector3d &);
  geometry_msgs::WrenchStamped msg_from_wrench(Eigen::Vector3d &, Eigen::Vector3d);

  // Math
  void append_to_history(controller_deque &history, Eigen::Vector3d &datum, unsigned int length);
  Eigen::AngleAxis<double> rotation_difference(Eigen::Matrix3d &rotation_a, Eigen::Matrix3d &rotation_b);
  Eigen::Vector3d estimate_derivative(controller_deque &);
  Eigen::Vector3d estimate_integral(controller_deque &history);

  // Control
  void control_loop(const ros::TimerEvent &);
};