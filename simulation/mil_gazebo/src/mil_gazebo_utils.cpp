#include <mil_gazebo/mil_gazebo_utils.hpp>
#include <gazebo/sensors/sensors.hh>

namespace mil_gazebo
{

void Convert(ignition::math::Vector3d const& _in, geometry_msgs::Vector3& _out)
{
  _out.x = _in.X();
  _out.y = _in.Y();
  _out.z = _in.Z();
}


void Convert(gazebo::common::Time const& _in, ros::Time& _out)
{
  _out.sec = _in.sec;
  _out.nsec = _in.nsec;
}

double NoiseCovariance(gazebo::sensors::Noise const& _noise)
{
  if (gazebo::sensors::Noise::GAUSSIAN == _noise.GetNoiseType())
  {
    double std_dev = dynamic_cast<gazebo::sensors::GaussianNoiseModel const&>(_noise).GetStdDev();
    return std_dev * std_dev;
  } else return 0.;
}


}
