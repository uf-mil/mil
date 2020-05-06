#include <mil_gazebo/mil_inclinometer_gazebo.hpp>

namespace mil_gazebo
{
// Register this plugin
GZ_REGISTER_SENSOR_PLUGIN(MilInclinometerGazebo)

MilInclinometerGazebo::MilInclinometerGazebo() : SensorPlugin()
{
}

MilInclinometerGazebo::~MilInclinometerGazebo()
{
}

void MilInclinometerGazebo::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(_parent);
  if (!sensor_)
  {
    ROS_ERROR_NAMED("MILInclinometerGazebo", "Parent sensor is null");
    return;
  }

  parent_ = boost::dynamic_pointer_cast<gazebo::physics::Link>(
      gazebo::physics::get_world(sensor_->WorldName())->EntityByName(sensor_->ParentName()));
  if (!parent_)
  {
    ROS_ERROR_NAMED("MILInclinometerGazebo", "Parent link is null");
    return;
  }

  if (_sdf->HasElement("frame_id"))
  {
    frame_name = _sdf->GetElement("frame_id")->Get<std::string>();
  }
  else
  {
    frame_name = sensor_->ParentName();
  }
    
  msg_.incremental_velocity_covariance[0] =
      NoiseCovariance(*sensor_->Noise(gazebo::sensors::SensorNoiseType::IMU_LINACC_X_NOISE_METERS_PER_S_SQR));
  msg_.incremental_velocity_covariance[4] =
      NoiseCovariance(*sensor_->Noise(gazebo::sensors::SensorNoiseType::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR));
  msg_.incremental_velocity_covariance[8] =
      NoiseCovariance(*sensor_->Noise(gazebo::sensors::SensorNoiseType::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR));

  pub_ = nh_.advertise<mil_msgs::IncrementalVelocity>("/imu/inclinometer", 20);

  update_connection_ = sensor_->ConnectUpdated(boost::bind(&MilInclinometerGazebo::OnUpdate, this));
}

void MilInclinometerGazebo::OnUpdate()
{
  if (pub_.getNumSubscribers())
  {
    msg_.header.frame_id = frame_name;
    Convert(sensor_->LastMeasurementTime(), msg_.header.stamp);

    current_velocity = parent_->RelativeLinearVel(); 
    incremental_velocity = current_velocity - previous_velocity;
    previous_velocity = current_velocity;
    Convert(incremental_velocity, msg_.incremental_velocity);

    pub_.publish(msg_);
  }
}
}