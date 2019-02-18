#ifndef MIL_IMU_GAZEBO_H
#define MIL_IMU_GAZEBO_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/sensors.hh>

namespace mil_gazebo
{

class MilImuGazebo : public gazebo::SensorPlugin
{
public:
  MilImuGazebo();
  ~MilImuGazebo();
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

private:
  void OnUpdate();
  ros::NodeHandle nh_;
  ros::Publisher imu_pub_;
  sensor_msgs::Imu msg_;
  gazebo::sensors::ImuSensorPtr sensor_;
  std::string frame_name_;
  gazebo::event::ConnectionPtr connection_;
};

}

#endif //MIL_IMU_GAZEBO_H
