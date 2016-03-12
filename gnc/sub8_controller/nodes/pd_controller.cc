/*
  This is a simple PD Controller designed for a few reasons:
    - Prove the validity of the existing simulator
      - And to serve as a tool for setting simulated waypoints without directly setting position
    - Prove the controller unit-test / monte-carlo system as a useful tool for testing
    - Experiment with C++ Dynamic Reconfigure
    - Start filling out the gnc/control stuff

  It is NOT:
    - Intended to be a serious replacement for the sub7 controller
        (Something else is cooking to replace that)
*/
#include "sub8_controller/pd_controller.h"

PDController::PDController() {
  reconfigure_callback = boost::bind(&PDController::gain_callback, this, _1, _2);
  server.setCallback(reconfigure_callback);
  // Wrench Publisher
  wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("wrench", 1);
  // Target Waypoint
  waypoint_sub = nh.subscribe("trajectory", 1, &PDController::trajectory_callback, this);
  // Get truth odometry
  truth_sub = nh.subscribe("odom", 1, &PDController::truth_callback, this);
  control_timer = nh.createTimer(ros::Duration(control_period), &PDController::control_loop, this);
}

void PDController::control_loop(const ros::TimerEvent &timer_event) {
  // Translation
  Eigen::Vector3d world_force;
  Eigen::Vector3d body_force;
  Eigen::Vector3d translation_error;
  Eigen::Vector3d translation_error_gradient;
  Eigen::Vector3d translation_error_integral;

  // Orientation
  Eigen::AngleAxis<double> orientation_error;
  double angle;
  Eigen::Vector3d angle_axis;
  Eigen::Vector3d orientation_error_gradient;
  Eigen::Vector3d orientation_error_integral;
  Eigen::Vector3d world_torque;
  Eigen::Vector3d body_torque;

  // Common
  geometry_msgs::WrenchStamped wrench_stamped_msg;
  Eigen::Matrix<double, 6, 1> body_wrench;

  if (not(got_state and got_target)) {
    // Wait until we have some data to work with
    return;
  }

  // ******* Translation Control *******
  translation_error = target_state.position - current_state.position;

  append_to_history(translation_error_history, translation_error, trans_history_length);

  translation_error_gradient = estimate_derivative(translation_error_history);
  translation_error_integral = estimate_integral(translation_error_history);

  // Convert error to body frame
  world_force = ((kp_trans * translation_error) + (kd_trans * translation_error_gradient) +
                 (ki_trans * translation_error_integral));

  // Feed-Forward vehicle weight (This drastically improves performance)
  world_force += sub_mass * 9.81 * Eigen::Vector3d::UnitZ();
  body_force = current_state.orientation * world_force;

  // ******* Orientation Control *******
  orientation_error = rotation_difference(current_state.orientation, target_state.orientation);

  // Normalize [-pi, pi]
  angle = orientation_error.angle();
  if (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }

  angle_axis = angle * orientation_error.axis();

  append_to_history(orientation_error_history, angle_axis, angle_history_length);
  orientation_error_gradient = estimate_derivative(orientation_error_history);
  orientation_error_integral = estimate_integral(orientation_error_history);

  world_torque = ((kp_angle * angle_axis) + (kd_angle * orientation_error_gradient) +
                   (ki_angle * orientation_error_integral));
  body_torque = current_state.orientation * world_torque;

  // ******* Transmit Wrenches *******
  body_wrench << body_force, body_torque;
  tf::wrenchEigenToMsg(body_wrench, wrench_stamped_msg.wrench);
  wrench_stamped_msg.header.frame_id = "/body";
  wrench_stamped_msg.header.stamp = ros::Time::now();
  wrench_pub.publish(wrench_stamped_msg);
}

void PDController::append_to_history(ControllerDeque &history, Eigen::Vector3d &datum,
                                     unsigned int length) {
  if (history.size() < length) {
    history.push_back(datum);
  } else if (history.size() > length) {
    history.pop_front();
  } else {
    // Remove oldest element
    history.pop_front();
    history.push_back(datum);
  }
}

Eigen::Vector3d PDController::estimate_integral(ControllerDeque &history) {
  /* Trapezoidal integration */
  Eigen::Vector3d integral;
  Eigen::Vector3d current;
  Eigen::Vector3d previous;

  for (ControllerDeque::iterator it = history.begin() + 1; it != history.end(); ++it) {
    current = *it;
    previous = *(it - 1);
    integral += (current + previous) * control_period / 2.0;
  }
  return integral;
}

Eigen::Vector3d PDController::estimate_derivative(ControllerDeque &history) {
  /* Forward difference to compute derivative */
  Eigen::Vector3d gradient;
  Eigen::Vector3d sum_difference;
  Eigen::Vector3d current;
  Eigen::Vector3d previous;

  for (ControllerDeque::iterator it = history.begin() + 1; it != history.end(); ++it) {
    current = *it;
    previous = *(it - 1);
    sum_difference += (current - previous) / control_period;
  }
  gradient = sum_difference / history.size();
  return gradient;
}

Eigen::AngleAxis<double> PDController::rotation_difference(Eigen::Matrix3d &rotation_a,
                                                           Eigen::Matrix3d &rotation_b) {
  Eigen::AngleAxis<double> angle_axis;
  Eigen::Matrix3d difference_matrix;

  difference_matrix = rotation_b * rotation_a.transpose();
  angle_axis.fromRotationMatrix(difference_matrix);
  return angle_axis;
}

void PDController::gain_callback(sub8_controller::GainConfig &config, uint32_t level) {
  /* Set gains using dynamic reconfigure */
  kp_trans = config.kp_trans;
  kd_trans = config.kd_trans;
  ki_trans = config.ki_trans;
  kp_angle = config.kp_angle;
  kd_angle = config.kd_angle;
  ki_angle = config.ki_angle;
  trans_history_length = config.history_length;
  sub_mass = config.sub_mass;
}

void PDController::truth_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  /* Callback on truth data */
  Eigen::Affine3d pose_matrix;
  Eigen::Matrix<double, 6, 1> twist;
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;

  got_state = true;

  tf::poseMsgToEigen(odom_msg->pose.pose, pose_matrix);
  current_state.position = pose_matrix.translation();
  current_state.orientation = pose_matrix.rotation();

  tf::twistMsgToEigen(odom_msg->twist.twist, twist);
  current_state.linear_velocity << current_state.orientation *(twist.head(3));
  current_state.angular_velocity << current_state.orientation *(twist.tail<3>());

  if (got_target) {
    if (waypoint_index == current_trajectory->trajectory.size() - 1) {
      // Hold at the last waypoint
      return;
    }

    // For now, only doing waypoint achievement on position error
    if ((current_state.position - target_state.position).norm() < waypoint_achievement_distance) {
      set_target(++waypoint_index);
    }
  }
}

void PDController::trajectory_callback(const sub8_msgs::Trajectory::ConstPtr &trajectory) {
  /* Callback on target trajectory */

  got_target = true;
  waypoint_index = 0;

  // Clear the error history
  current_trajectory = trajectory;
  set_target(waypoint_index);
}

void PDController::set_target(unsigned int waypoint_index) {
  /* Callback on target waypoint */
  Eigen::Affine3d target_pose_matrix;
  Eigen::Matrix<double, 6, 1> twist;
  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;

  translation_error_history.erase(translation_error_history.begin(),
                                  translation_error_history.end());
  orientation_error_history.erase(orientation_error_history.begin(),
                                  orientation_error_history.end());

  tf::poseMsgToEigen(current_trajectory->trajectory[waypoint_index].pose, target_pose_matrix);
  target_state.position = target_pose_matrix.translation();
  target_state.orientation = target_pose_matrix.rotation();

  if (target_state.orientation.norm() < 0.01) {
    // If somebody passes a target with no orientation set, do something that makes sense
    if (got_state) {
      // NEW: Transpose, not sureif good
      target_state.orientation = current_state.orientation;
    } else {
      target_state.orientation = Eigen::Affine3d::Identity().rotation();
    }
  }

  tf::twistMsgToEigen(current_trajectory->trajectory[waypoint_index].twist, twist);
  target_state.linear_velocity << twist.head(3);
  target_state.angular_velocity << twist.tail<3>();
}

int main(int argc, char **argv) {
  ROS_INFO("Starting up controller");
  ros::init(argc, argv, "pd_controller");
  PDController pdc;
  ROS_INFO("Spinning");
  ros::spin();
  return 0;
}