#include <mil_gazebo/mil_magnetometer_gazebo.hpp>

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
    auto field = sensor_->MagneticField();
    sensor_msgs::MagneticField msg;
    msg.magnetic_field.x =  field.X();
    msg.magnetic_field.y =  field.Y();
    msg.magnetic_field.z =  field.Z();
    msg.header.frame_id = frame_name_;
    msg.header.stamp.sec = sensor_->LastMeasurementTime().sec;
    msg.header.stamp.nsec = sensor_->LastMeasurementTime().nsec;
    mag_pub_.publish(msg);
  }
}
}
