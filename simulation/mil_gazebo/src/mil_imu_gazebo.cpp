#include <mil_gazebo/mil_imu_gazebo.hpp>
#include <mil_gazebo/mil_gazebo_utils.hpp>

namespace mil_gazebo
{

GZ_REGISTER_SENSOR_PLUGIN(MilImuGazebo)

MilImuGazebo::MilImuGazebo(): SensorPlugin()
{
}


MilImuGazebo::~MilImuGazebo()
{
}


void MilImuGazebo::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  sensor_ = std::dynamic_pointer_cast<gazebo::sensors::ImuSensor>(_parent);
  if (!sensor_)
  {
    ROS_ERROR_NAMED("MilImuGazebo", "Parent sensor is null");
    return;
  }

  if(_sdf->HasElement("frame_id"))
    frame_name_ = _sdf->GetElement("frame_id")->Get<std::string>();
  else frame_name_ = sensor_->ParentName();

  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw", 20);

  // Initialize message with correct covariances and header frame
  msg_.header.frame_id = frame_name_;
  msg_.angular_velocity_covariance[0] = NoiseCovariance(*sensor_->Noise(gazebo::sensors::SensorNoiseType::IMU_ANGVEL_X_NOISE_RADIANS_PER_S));
  msg_.angular_velocity_covariance[4] = NoiseCovariance(*sensor_->Noise(gazebo::sensors::SensorNoiseType::IMU_ANGVEL_Y_NOISE_RADIANS_PER_S));
  msg_.angular_velocity_covariance[8] = NoiseCovariance(*sensor_->Noise(gazebo::sensors::SensorNoiseType::IMU_ANGVEL_Z_NOISE_RADIANS_PER_S));
  msg_.linear_acceleration_covariance[0] = NoiseCovariance(*sensor_->Noise(gazebo::sensors::SensorNoiseType::IMU_LINACC_X_NOISE_METERS_PER_S_SQR));
  msg_.linear_acceleration_covariance[4] = NoiseCovariance(*sensor_->Noise(gazebo::sensors::SensorNoiseType::IMU_LINACC_Y_NOISE_METERS_PER_S_SQR));
  msg_.linear_acceleration_covariance[8] = NoiseCovariance(*sensor_->Noise(gazebo::sensors::SensorNoiseType::IMU_LINACC_Z_NOISE_METERS_PER_S_SQR));

  connection_ = sensor_->ConnectUpdated(boost::bind(&MilImuGazebo::OnUpdate, this));
}

void MilImuGazebo::OnUpdate()
{
  if (!imu_pub_.getNumSubscribers())
    return;

  Convert(sensor_->LastMeasurementTime(), msg_.header.stamp);
  Convert(sensor_->AngularVelocity(), msg_.angular_velocity);
  Convert(sensor_->LinearAcceleration(), msg_.linear_acceleration);

  imu_pub_.publish(msg_);
}

} // namespace mil_gazebo
