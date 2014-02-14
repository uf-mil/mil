#ifndef GUARD_XKTOCTGPUKGKWLWG
#define GUARD_XKTOCTGPUKGKWLWG

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <odom_estimator/unscented_transform.h>

namespace odom_estimator {


struct Odom {
  ros::Time stamp;
  std::string frame_id;
  std::string child_frame_id;
  
  Vec<3> pos;
  Quaternion orient;
  Vec<3> vel;
  Vec<3> ang_vel;
  static const int RowsAtCompileTime = 3*4;
  unsigned int rows() const {
    return RowsAtCompileTime;
  }
  
  Odom(ros::Time stamp,
       std::string frame_id, std::string child_frame_id,
       Vec<3> pos, Quaternion orient,
       Vec<3> vel, Vec<3> ang_vel) :
    stamp(stamp),
    frame_id(frame_id), child_frame_id(child_frame_id),
    pos(pos), orient(orient.normalized()),
    vel(vel), ang_vel(ang_vel) {
  }
  
  Vec<RowsAtCompileTime> operator-(const Odom &other) const {
    assert(other.stamp == stamp);
    assert(other.frame_id == frame_id);
    assert(other.child_frame_id == child_frame_id);
    return (Vec<RowsAtCompileTime>() <<
      pos - other.pos,
      rotvec_from_quat(orient * other.orient.conjugate()),
      vel - other.vel,
      ang_vel - other.ang_vel).finished();
  }
  Odom operator+(const Vec<RowsAtCompileTime> &other) const {
    return Odom(
      stamp,
      frame_id,
      child_frame_id,
      pos + other.segment<3>(0),
      quat_from_rotvec(other.segment<3>(3)) * orient,
      vel + other.segment<3>(6),
      ang_vel + other.segment<3>(9));
  }
};


template<typename ArgType, typename ResType>
ResType functional(void func(ArgType const &, ResType &),
                   ArgType const &arg) {
  ResType res;
  func(arg, res);
  return res;
}

GaussianDistribution<Odom> odom_from_msg(nav_msgs::Odometry const &msg) {
  return GaussianDistribution<Odom>(
    Odom(
      msg.header.stamp,
      msg.header.frame_id,
      msg.child_frame_id,
      functional(tf::pointMsgToEigen, msg.pose.pose.position),
      functional(tf::quaternionMsgToEigen, msg.pose.pose.orientation),
      functional(tf::vectorMsgToEigen, msg.twist.twist.linear),
      functional(tf::vectorMsgToEigen, msg.twist.twist.angular)),
    (SqMat<12>() << 
      Eigen::Map<const SqMat<6> >(msg.pose.covariance.data()),
        SqMat<6>::Zero(),
      SqMat<6>::Zero(),
        Eigen::Map<const SqMat<6> >(msg.twist.covariance.data())).finished());
}

nav_msgs::Odometry msg_from_odom(GaussianDistribution<Odom> const &res) {
  nav_msgs::Odometry result;
  result.header.stamp = res.mean.stamp;
  result.header.frame_id = res.mean.frame_id;
  result.child_frame_id = res.mean.child_frame_id;
  
  tf::pointEigenToMsg(res.mean.pos, result.pose.pose.position);
  tf::quaternionEigenToMsg(res.mean.orient, result.pose.pose.orientation);
  Eigen::Map<SqMat<6> >(result.pose.covariance.data()) = res.cov.block<6, 6>(0, 0);
  
  tf::vectorEigenToMsg(res.mean.vel, result.twist.twist.linear);
  tf::vectorEigenToMsg(res.mean.ang_vel, result.twist.twist.angular);
  Eigen::Map<SqMat<6> >(result.twist.covariance.data()) = res.cov.block<6, 6>(6, 6);
  
  return result;
}


}

#endif
