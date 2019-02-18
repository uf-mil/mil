#include <mil_gazebo/mil_dvl_gazebo.hpp>

namespace mil_gazebo
{
// Register this plugin
GZ_REGISTER_SENSOR_PLUGIN(MilDVLGazebo)

MilDVLGazebo::MilDVLGazebo() : SensorPlugin()
{
}

MilDVLGazebo::~MilDVLGazebo()
{
}

void MilDVLGazebo::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  sensor_ = std::dynamic_pointer_cast<gazebo::sensors::RaySensor>(_parent);
  if (!sensor_)
  {
    ROS_ERROR_NAMED("MILDVLGazebo", "Parent sensor is null");
    return;
  }

  parent_ = gazebo::physics::get_world(sensor_->WorldName())->GetEntity(sensor_->ParentName());
  if (!parent_)
  {
    ROS_ERROR_NAMED("MILDVLGazebo", "Parent is null");
    return;
  }

  pose_ = sensor_->Pose();
  auto new_pose = ignition::math::Pose3d(
      pose_.Pos(), pose_.Rot() * ignition::math::Quaterniond(0., ignition::math::Angle::HalfPi.Radian(), 0.));
  sensor_->SetPose(new_pose);

  if (_sdf->HasElement("frame_id"))
  {
    frame_name_ = _sdf->GetElement("frame_id")->Get<std::string>();
  }
  else
    frame_name_ = sensor_->ParentName();

  vel_pub_ = nh_.advertise<mil_msgs::VelocityMeasurements>("/dvl", 20);
  range_pub_ = nh_.advertise<mil_msgs::RangeStamped>("/dvl/range", 20);

  update_connection_ = sensor_->ConnectUpdated(boost::bind(&MilDVLGazebo::OnUpdate, this));
}

void MilDVLGazebo::OnUpdate()
{
  if (range_pub_.getNumSubscribers())
  {
    double range = sensor_->Range(0);
    mil_msgs::RangeStamped range_msg;
    range_msg.header.frame_id = frame_name_;
    range_msg.header.stamp.sec = sensor_->LastMeasurementTime().sec;
    range_msg.header.stamp.nsec = sensor_->LastMeasurementTime().nsec;
    range_msg.range = range;
    range_pub_.publish(range_msg);
  }

  if (vel_pub_.getNumSubscribers())
  {
    mil_msgs::VelocityMeasurements vel_msg;
    vel_msg.header.frame_id = frame_name_;
    vel_msg.header.stamp.sec = sensor_->LastMeasurementTime().sec;
    vel_msg.header.stamp.nsec = sensor_->LastMeasurementTime().nsec;

    ignition::math::Vector3d vel = pose_.Rot().Inverse().RotateVector(parent_->GetRelativeLinearVel().Ign());

    double tilt = 30 * ignition::math::Angle::Pi.Radian() / 180;
    double x = sin(tilt);
    double z = cos(tilt);

    std::array<ignition::math::Vector3d, 4> directions = {
      ignition::math::Vector3d(-x, 0., -z), ignition::math::Vector3d(x, 0., -z), ignition::math::Vector3d(0., x, -z),
      ignition::math::Vector3d(0., -x, -z),
    };

    for (auto dir : directions)
    {
      mil_msgs::VelocityMeasurement measurement;
      measurement.direction.x = dir.X();
      measurement.direction.y = dir.Y();
      measurement.direction.z = dir.Z();
      measurement.velocity = vel.Dot(dir);
      measurement.correlation = ignition::math::NAN_D;
      vel_msg.velocity_measurements.push_back(measurement);
    }

    vel_pub_.publish(vel_msg);
  }
}
}
