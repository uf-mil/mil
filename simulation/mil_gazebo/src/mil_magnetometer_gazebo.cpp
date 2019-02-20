#include <mil_gazebo/mil_magnetometer_gazebo.hpp>
#include <mil_gazebo/mil_gazebo_utils.hpp>

namespace mil_gazebo
{

// Register this plugin
GZ_REGISTER_SENSOR_PLUGIN(MilMagnetometerGazebo)

MilMagnetometerGazebo::MilMagnetometerGazebo() : SensorPlugin()
{
}

MilMagnetometerGazebo::~MilMagnetometerGazebo()
{
}

void MilMagnetometerGazebo::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  sensor_ = std::dynamic_pointer_cast<gazebo::sensors::MagnetometerSensor>(_parent);
  if (!sensor_)
  {
    ROS_ERROR_NAMED("MilMagnetometerGazebo", "Parent sensor is null");
    return;
  }

  if (_sdf->HasElement("frame_id"))
  {
    frame_name_ = _sdf->GetElement("frame_id")->Get<std::string>();
  }
  else
    frame_name_ = sensor_->ParentName();

  mag_pub_ = nh_.advertise<sensor_msgs::MagneticField>("/imu/mag", 20);

  update_connection_ = sensor_->ConnectUpdated(boost::bind(&MilMagnetometerGazebo::OnUpdate, this));
}

void MilMagnetometerGazebo::OnUpdate()
{
  if (mag_pub_.getNumSubscribers())
  {
    sensor_msgs::MagneticField msg;
    Convert(sensor_->MagneticField(), msg.magnetic_field);
    msg.header.frame_id = frame_name_;
    Convert(sensor_->LastUpdateTime(), msg.header.stamp);
    mag_pub_.publish(msg);
  }
}
}
