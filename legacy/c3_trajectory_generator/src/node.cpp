#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <mil_msgs/PoseTwistStamped.h>
#include <mil_tools/msg_helpers.hpp>
#include <mil_tools/param_helpers.hpp>
#include <ros_alarms/listener.hpp>

#include <mil_msgs/MoveToAction.h>
#include "C3Trajectory.h"
#include "c3_trajectory_generator/SetDisabled.h"

#include <waypoint_validity.hpp>

#include <boost/assign/list_of.hpp>
#include <boost/unordered_map.hpp>

using namespace std;
using namespace geometry_msgs;
using namespace nav_msgs;
using namespace mil_msgs;
using namespace mil_tools;
using namespace c3_trajectory_generator;

const boost::unordered_map<WAYPOINT_ERROR_TYPE, const std::string> WAYPOINT_ERROR_TO_STRING =
    boost::assign::map_list_of(WAYPOINT_ERROR_TYPE::OCCUPIED, "OCCUPIED")(WAYPOINT_ERROR_TYPE::UNKNOWN, "UNKNOWN")(
        WAYPOINT_ERROR_TYPE::UNOCCUPIED, "UNOCCUPIED")(WAYPOINT_ERROR_TYPE::ABOVE_WATER, "ABOVE_WATER")(
        WAYPOINT_ERROR_TYPE::NO_OGRID, "NO_OGRID")(WAYPOINT_ERROR_TYPE::NOT_CHECKED, "NOT_CHECKED")(
        WAYPOINT_ERROR_TYPE::OCCUPIED_TRAJECTORY, "OCCUPIED_TRAJECTORY");

subjugator::C3Trajectory::Point Point_from_PoseTwist(const Pose &pose, const Twist &twist)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation, q);

  subjugator::C3Trajectory::Point res;

  res.q.head(3) = xyz2vec(pose.position);
  tf::Matrix3x3(q).getRPY(res.q[3], res.q[4], res.q[5]);

  // clang-format off
  res.qdot.head(3) = vec2vec(tf::Matrix3x3(q) * vec2vec(xyz2vec(twist.linear)));
  res.qdot.tail(3) = (Eigen::Matrix3d() << 1, sin(res.q[3]) * tan(res.q[4]),
                        cos(res.q[3]) * tan(res.q[4]), 0, cos(res.q[3]), -sin(res.q[3]), 0,
                        sin(res.q[3]) / cos(res.q[4]), cos(res.q[3]) / cos(res.q[4])).finished() *
                      xyz2vec(twist.angular);
  // clang-format on
  return res;
}

PoseTwist PoseTwist_from_PointWithAcceleration(const subjugator::C3Trajectory::PointWithAcceleration &p)
{
  tf::Quaternion orient = tf::createQuaternionFromRPY(p.q[3], p.q[4], p.q[5]);

  PoseTwist res;

  res.pose.position = vec2xyz<Point>(p.q.head(3));
  quaternionTFToMsg(orient, res.pose.orientation);

  Eigen::Matrix3d worldangvel_from_eulerrates = (Eigen::Matrix3d() << 1, 0, -sin(p.q[4]), 0, cos(p.q[3]),
                                                 sin(p.q[3]) * cos(p.q[4]), 0, -sin(p.q[3]), cos(p.q[3]) * cos(p.q[4]))
                                                    .finished();

  res.twist.linear = vec2xyz<Vector3>(tf::Matrix3x3(orient.inverse()) * vec2vec(p.qdot.head(3)));
  res.twist.angular = vec2xyz<Vector3>(worldangvel_from_eulerrates * p.qdot.tail(3));

  res.acceleration.linear = vec2xyz<Vector3>(tf::Matrix3x3(orient.inverse()) * vec2vec(p.qdotdot.head(3)));
  res.acceleration.angular = vec2xyz<Vector3>(worldangvel_from_eulerrates * p.qdotdot.tail(3));

  return res;
}

Pose Pose_from_Waypoint(const subjugator::C3Trajectory::Waypoint &wp)
{
  Pose res;

  res.position = vec2xyz<Point>(wp.r.q.head(3));
  quaternionTFToMsg(tf::createQuaternionFromRPY(wp.r.q[3], wp.r.q[4], wp.r.q[5]), res.orientation);

  return res;
}

struct Node
{
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  tf::TransformListener tf_listener;
  ros_alarms::AlarmListener<> kill_listener;

  string body_frame;
  string fixed_frame;
  subjugator::C3Trajectory::Limits limits;
  ros::Duration traj_dt;

  ros::Subscriber odom_sub;
  actionlib::SimpleActionServer<mil_msgs::MoveToAction> actionserver;
  ros::Publisher trajectory_pub;
  ros::Publisher trajectory_vis_pub;
  ros::Publisher waypoint_pose_pub;
  ros::ServiceServer set_disabled_service;

  ros::Timer update_timer;

  bool disabled;
  boost::scoped_ptr<subjugator::C3Trajectory> c3trajectory;
  ros::Time c3trajectory_t;

  subjugator::C3Trajectory::Waypoint current_waypoint;
  ros::Time current_waypoint_t;

  double linear_tolerance, angular_tolerance;

  WaypointValidity waypoint_validity_;
  bool waypoint_check_;

  bool set_disabled(SetDisabledRequest &request, SetDisabledResponse &response)
  {
    disabled = request.disabled;
    if (disabled)
    {
      c3trajectory.reset();
    }
    return true;
  }

  Node()
    : private_nh("~")
    , actionserver(nh, "moveto", false)
    , disabled(false)
    , kill_listener(nh, "kill")
    , waypoint_validity_(nh)
  {
    // Make sure alarm integration is ok
    kill_listener.waitForConnection(ros::Duration(2));
    if (kill_listener.getNumConnections() < 1)
      throw std::runtime_error("The kill listener isn't connected to the alarm server");
    kill_listener.start();  // Fuck.

    fixed_frame = mil_tools::getParam<std::string>(private_nh, "fixed_frame");
    body_frame = mil_tools::getParam<std::string>(private_nh, "body_frame");

    limits.vmin_b = mil_tools::getParam<subjugator::Vector6d>(private_nh, "vmin_b");
    limits.vmax_b = mil_tools::getParam<subjugator::Vector6d>(private_nh, "vmax_b");
    limits.amin_b = mil_tools::getParam<subjugator::Vector6d>(private_nh, "amin_b");
    limits.amax_b = mil_tools::getParam<subjugator::Vector6d>(private_nh, "amax_b");
    limits.arevoffset_b = mil_tools::getParam<Eigen::Vector3d>(private_nh, "arevoffset_b");
    limits.umax_b = mil_tools::getParam<subjugator::Vector6d>(private_nh, "umax_b");
    traj_dt = mil_tools::getParam<ros::Duration>(private_nh, "traj_dt", ros::Duration(0.0001));

    waypoint_check_ = mil_tools::getParam<bool>(private_nh, "waypoint_check");

    odom_sub = nh.subscribe<Odometry>("odom", 1, boost::bind(&Node::odom_callback, this, _1));

    trajectory_pub = nh.advertise<PoseTwistStamped>("trajectory", 1);
    trajectory_vis_pub = private_nh.advertise<PoseStamped>("trajectory_v", 1);
    waypoint_pose_pub = private_nh.advertise<PoseStamped>("waypoint", 1);

    update_timer = nh.createTimer(ros::Duration(1. / 50), boost::bind(&Node::timer_callback, this, _1));

    actionserver.start();

    set_disabled_service = private_nh.advertiseService<SetDisabledRequest, SetDisabledResponse>(
        "set_disabled", boost::bind(&Node::set_disabled, this, _1, _2));
  }

  void odom_callback(const OdometryConstPtr &odom)
  {
    if (c3trajectory)
      return;  // already initialized
    if (kill_listener.isRaised() || disabled)
      return;  // only initialize when unkilled

    ros::Time now = ros::Time::now();

    subjugator::C3Trajectory::Point current = Point_from_PoseTwist(odom->pose.pose, odom->twist.twist);
    current.q[3] = current.q[4] = 0;              // zero roll and pitch
    current.qdot = subjugator::Vector6d::Zero();  // zero velocities

    c3trajectory.reset(new subjugator::C3Trajectory(current, limits));
    c3trajectory_t = now;

    current_waypoint = current;
    current_waypoint_t = now;
  }

  void timer_callback(const ros::TimerEvent &)
  {
    mil_msgs::MoveToResult actionresult;

    // Handle disabled, killed, or no odom before attempting to produce trajectory
    std::string err = "";
    if (disabled)
      err = "c3 disabled";
    else if (kill_listener.isRaised())
      err = "killed";
    else if (!c3trajectory)
      err = "no odom";

    if (!err.empty())
    {
      if (c3trajectory)
        c3trajectory.reset();  // On revive/enable, wait for odom before station keeping

      // Cancel all goals while killed/disabled/no odom
      if (actionserver.isNewGoalAvailable())
        actionserver.acceptNewGoal();
      if (actionserver.isActive())
      {
        actionresult.error = err;
        actionresult.success = false;
        actionserver.setAborted(actionresult);
      }
      return;
    }

    ros::Time now = ros::Time::now();

    auto old_waypoint = current_waypoint;

    if (actionserver.isNewGoalAvailable())
    {
      boost::shared_ptr<const mil_msgs::MoveToGoal> goal = actionserver.acceptNewGoal();
      current_waypoint =
          subjugator::C3Trajectory::Waypoint(Point_from_PoseTwist(goal->posetwist.pose, goal->posetwist.twist),
                                             goal->speed, !goal->uncoordinated, !goal->blind);
      current_waypoint_t = now;
      this->linear_tolerance = goal->linear_tolerance;
      this->angular_tolerance = goal->angular_tolerance;

      waypoint_validity_.pub_size_ogrid(Pose_from_Waypoint(current_waypoint), (int)OGRID_COLOR::GREEN);

      // Check if waypoint is valid
      std::pair<bool, WAYPOINT_ERROR_TYPE> checkWPResult = waypoint_validity_.is_waypoint_valid(
          Pose_from_Waypoint(current_waypoint), current_waypoint.do_waypoint_validation);
      actionresult.error = WAYPOINT_ERROR_TO_STRING.at(checkWPResult.second);
      actionresult.success = checkWPResult.first;
      if (checkWPResult.first == false && waypoint_check_)  // got a point that we should not move to
      {
        waypoint_validity_.pub_size_ogrid(Pose_from_Waypoint(current_waypoint), (int)OGRID_COLOR::RED);
        if (checkWPResult.second ==
            WAYPOINT_ERROR_TYPE::UNKNOWN)  // if unknown, check if there's a huge displacement with the new waypoint
        {
          auto a_point = Pose_from_Waypoint(current_waypoint);
          auto b_point = Pose_from_Waypoint(old_waypoint);
          // If moved more than .5m, then don't allow
          if (abs(a_point.position.x - b_point.position.x) > .5 || abs(a_point.position.y - b_point.position.y) > .5)
          {
            ROS_ERROR("can't move there! - need to rotate");
            current_waypoint = old_waypoint;
          }
        }
        // if point is occupied, reject move
        if (checkWPResult.second == WAYPOINT_ERROR_TYPE::OCCUPIED)
        {
          ROS_ERROR("can't move there! - waypoint is occupied");
          current_waypoint = old_waypoint;
        }
        // if point is above water, reject move
        if (checkWPResult.second == WAYPOINT_ERROR_TYPE::ABOVE_WATER)
        {
          ROS_ERROR("can't move there! - waypoint is above water");
          current_waypoint = old_waypoint;
        }
        if (checkWPResult.second == WAYPOINT_ERROR_TYPE::NO_OGRID)
        {
          ROS_ERROR("WaypointValidity - Did not recieve any ogrid");
        }
      }
    }
    if (actionserver.isPreemptRequested())
    {
      current_waypoint = c3trajectory->getCurrentPoint();
      current_waypoint.do_waypoint_validation = false;
      current_waypoint.r.qdot = subjugator::Vector6d::Zero();  // zero velocities
      current_waypoint_t = now;

      // don't try to make output c3 continuous when cancelled - instead stop as quickly as possible
      c3trajectory.reset(new subjugator::C3Trajectory(current_waypoint.r, limits));
      c3trajectory_t = now;
    }

    // Remember the previous trajectory
    auto old_trajectory = c3trajectory->getCurrentPoint();

    while (c3trajectory_t + traj_dt < now)
    {
      c3trajectory->update(traj_dt.toSec(), current_waypoint, (c3trajectory_t - current_waypoint_t).toSec());
      c3trajectory_t += traj_dt;
    }

    // Check if we will hit something while in trajectory the new trajectory
    geometry_msgs::Pose traj_point;  // Convert messages to correct type
    auto p = c3trajectory->getCurrentPoint();
    traj_point.position = vec2xyz<Point>(p.q.head(3));
    quaternionTFToMsg(tf::createQuaternionFromRPY(p.q[3], p.q[4], p.q[5]), traj_point.orientation);

    std::pair<bool, WAYPOINT_ERROR_TYPE> checkWPResult =
        waypoint_validity_.is_waypoint_valid(Pose_from_Waypoint(p), c3trajectory->do_waypoint_validation);

    if (checkWPResult.first == false && checkWPResult.second == WAYPOINT_ERROR_TYPE::OCCUPIED && waypoint_check_)
    {  // New trajectory will hit an occupied goal, so reject
      ROS_ERROR("can't move there! - bad trajectory");
      current_waypoint = old_trajectory;
      current_waypoint.do_waypoint_validation = false;
      current_waypoint.r.qdot = subjugator::Vector6d::Zero();  // zero velocities
      current_waypoint_t = now;

      c3trajectory.reset(new subjugator::C3Trajectory(current_waypoint.r, limits));
      c3trajectory_t = now;
      actionresult.success = false;
      actionresult.error = WAYPOINT_ERROR_TO_STRING.at(WAYPOINT_ERROR_TYPE::OCCUPIED_TRAJECTORY);
    }

    PoseTwistStamped msg;
    msg.header.stamp = c3trajectory_t;
    msg.header.frame_id = fixed_frame;
    msg.posetwist = PoseTwist_from_PointWithAcceleration(c3trajectory->getCurrentPoint());
    trajectory_pub.publish(msg);

    waypoint_validity_.pub_size_ogrid(Pose_from_Waypoint(c3trajectory->getCurrentPoint()), 200);

    PoseStamped msgVis;
    msgVis.header = msg.header;
    msgVis.pose = msg.posetwist.pose;
    trajectory_vis_pub.publish(msgVis);

    PoseStamped posemsg;
    posemsg.header.stamp = c3trajectory_t;
    posemsg.header.frame_id = fixed_frame;
    posemsg.pose = Pose_from_Waypoint(current_waypoint);
    waypoint_pose_pub.publish(posemsg);

    if (actionserver.isActive() &&
        c3trajectory->getCurrentPoint().is_approximately(current_waypoint.r, max(1e-3, linear_tolerance),
                                                         max(1e-3, angular_tolerance)) &&
        current_waypoint.r.qdot == subjugator::Vector6d::Zero())
    {
      actionresult.error = "";
      actionresult.success = true;
      actionserver.setSucceeded(actionresult);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "c3_trajectory_generator");

  Node n;

  ros::spin();

  return 0;
}
