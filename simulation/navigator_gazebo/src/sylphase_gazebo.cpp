#include "navigator_gazebo/sylphase_gazebo.hpp"

namespace navigator_gazebo
{
GZ_REGISTER_MODEL_PLUGIN(SylphaseGazebo)

SylphaseGazebo::SylphaseGazebo()
{
}

void SylphaseGazebo::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  child_frame_ = "ins";
  if (_sdf->HasElement("child_frame"))
    child_frame_ = _sdf->Get<std::string>("child_frame");
  else
    ROS_WARN_NAMED("sylphaze", "No child_frame specified, defaulint to %s", child_frame_.c_str());

  std::string odom_topic = "odom";
  if (_sdf->HasElement("odom_topic"))
    odom_topic = _sdf->Get<std::string>("odom_topic");

  std::string absodom_topic = "absodom";
  if (_sdf->HasElement("absodom_topic"))
    absodom_topic = _sdf->Get<std::string>("absodom_topic");

  std::string acceleration_topic = "acceleration";
  if (_sdf->HasElement("acceleration_topic"))
    acceleration_topic = _sdf->Get<std::string>("acceleration_topic");

  double update_hz = 30.;
  if (_sdf->HasElement("update_rate"))
    update_hz = _sdf->Get<double>("update_rate");
  update_period_ = 1.0 / update_hz;

  model_ = _model;

  pose_ = _sdf->Get<ignition::math::Pose3d>("pose");

  // Get tranformation from gazebo to enu and gazeb oto ecef
  converter_ = _model->GetWorld()->GetSphericalCoordinates();
  if (!converter_)
  {
    ROS_ERROR_NAMED("sylphase", "converter is null");
    return;
  }
  local_to_enu_ = SphericalCoordinatesTransform(*converter_, CoordinateType::LOCAL, CoordinateType::GLOBAL);
  local_to_ecef_ = SphericalCoordinatesTransform(*converter_, CoordinateType::LOCAL, CoordinateType::ECEF);

  // Output publishers
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic, 10);
  absodom_pub_ = nh_.advertise<nav_msgs::Odometry>(absodom_topic, 10);
  acceleration_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("acceleration", 10);

  update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&SylphaseGazebo::OnUpdate, this, std::placeholders::_1));
}

ignition::math::Matrix4d SylphaseGazebo::SphericalCoordinatesTransform(
    const gazebo::common::SphericalCoordinates& _converter, const CoordinateType in, const CoordinateType out)
{
  ignition::math::Vector3d origin = _converter.PositionTransform(ignition::math::Vector3d::Zero, in, out);
  ignition::math::Vector3d x = _converter.PositionTransform(origin + ignition::math::Vector3d::UnitX, in, out) - origin;
  ignition::math::Vector3d y = _converter.PositionTransform(origin + ignition::math::Vector3d::UnitY, in, out) - origin;
  ignition::math::Vector3d z = _converter.PositionTransform(origin + ignition::math::Vector3d::UnitZ, in, out) - origin;

  ignition::math::Matrix3d m;
  m.Axes(x, y, z);
  ignition::math::Quaterniond quat(m);

  return ignition::math::Matrix4d(ignition::math::Pose3d(origin, quat));
}

geometry_msgs::Quaternion SylphaseGazebo::Convert(const ignition::math::Quaterniond& _in)
{
  geometry_msgs::Quaternion out;
  out.x = _in.X();
  out.y = _in.Y();
  out.z = _in.Z();
  out.w = _in.W();
  return out;
}

geometry_msgs::Pose SylphaseGazebo::Convert(const ignition::math::Pose3d& _in)
{
  geometry_msgs::Pose pose;
  pose.position = Convert(_in.Pos());
  pose.orientation = Convert(_in.Rot());
  return pose;
}

geometry_msgs::Point SylphaseGazebo::Convert(const ignition::math::Vector3d& _in)
{
  geometry_msgs::Point out;
  out.x = _in.X();
  out.y = _in.Y();
  out.z = _in.Z();
  return out;
}

geometry_msgs::Vector3 SylphaseGazebo::Convert2(const ignition::math::Vector3d& _in)
{
  geometry_msgs::Vector3 out;
  out.x = _in.X();
  out.y = _in.Y();
  out.z = _in.Z();
  return out;
}

void SylphaseGazebo::OnUpdate(const gazebo::common::UpdateInfo& info)
{
  if ((info.simTime - last_update_time_).Double() < update_period_)
    return;
  last_update_time_ = info.simTime;

  // TODO: throttle
  auto pose = model_->GetWorldPose().Ign() + pose_;
  auto rot = pose_.Rot();
  auto vel_linear = rot * model_->GetRelativeLinearVel().Ign();
  auto accel_linear = rot * model_->GetRelativeLinearAccel().Ign();
  auto vel_angular = rot * model_->GetRelativeAngularVel().Ign();
  auto enu = (local_to_enu_ * pose).Pose();
  auto ecef = (local_to_ecef_ * pose).Pose();

  nav_msgs::Odometry odom;
  odom.header.frame_id = "enu";
  odom.header.stamp.sec = info.simTime.sec;
  odom.header.stamp.nsec = info.simTime.nsec;
  odom.child_frame_id = "ins";
  odom.pose.pose = Convert(enu);
  odom.twist.twist.linear = Convert2(vel_linear);
  odom.twist.twist.angular = Convert2(vel_angular);

  nav_msgs::Odometry absodom;
  absodom.pose.pose = Convert(ecef);
  absodom.twist.twist = odom.twist.twist;
  absodom.header.frame_id = "ecef";
  absodom.header.stamp = odom.header.stamp;

  geometry_msgs::Vector3Stamped accel;
  accel.header.stamp = odom.header.stamp;
  accel.header.frame_id = odom.header.frame_id;
  accel.vector = Convert2(vel_angular);

  odom_pub_.publish(odom);
  absodom_pub_.publish(absodom);
  acceleration_pub_.publish(accel);
}
}
