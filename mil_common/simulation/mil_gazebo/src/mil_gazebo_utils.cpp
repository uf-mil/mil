#include <ros/ros.h>
#include <gazebo/sensors/sensors.hh>
#include <mil_gazebo/mil_gazebo_utils.hpp>

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
  }
  else
    return 0.;
}

//This function is based on the technique used in Gazebo 11's GaussianNoiseModel Class to simulate random walk bias
//Link: https://github.com/osrf/gazebo/blob/gazebo11/gazebo/sensors/GaussianNoiseModel.cc
double addRandomWalkNoise(double& input, double& dt, double& previous_bias, double& random_walk, double& correlation_time) 
{
  double sigmaB = random_walk;
  double tau = correlation_time;
  double sigmaBD = sqrt(-sigmaB * sigmaB * tau / 2 * expm1(-2 * dt / tau));
  double phiD = exp(-dt / tau);
  double newBias = phiD * previous_bias + ignition::math::Rand::DblNormal(0, sigmaBD);
  previous_bias = newBias;

  return input + newBias;
}

int TagInSDFOrRosParam(sdf::ElementPtr _sdf, std::string const& tag)
{
  if (_sdf->HasElement(tag))
    return 0;
  else if (_sdf->HasElement(tag + "_param"))
  {
    std::string param_name = _sdf->GetElement(tag + "_param")->Get<std::string>();
    if (ros::param::has(param_name))
      return 1;
    else
      return -1;
  }
  else
    return -1;
}

bool GetFromSDFOrRosParam(sdf::ElementPtr _sdf, std::string const& tag, double& val)
{
  int mode = TagInSDFOrRosParam(_sdf, tag);

  if (mode == 0)
  {
    val = _sdf->GetElement(tag)->Get<double>();
  }
  else if (mode == 1)
  {
    std::string param_name = _sdf->GetElement(tag + "_param")->Get<std::string>();
    return ros::param::get(param_name, val);
  }
  else
    return false;
}

bool GetFromSDFOrRosParam(sdf::ElementPtr _sdf, std::string const& tag, ignition::math::Vector3d& val)
{
  int mode = TagInSDFOrRosParam(_sdf, tag);
  if (mode == 0)
  {
    val = _sdf->GetElement(tag)->Get<ignition::math::Vector3d>();
  }
  else if (mode == 1)
  {
    std::string param_name = _sdf->GetElement(tag + "_param")->Get<std::string>();
    std::vector<double> tmp;
    if (!ros::param::get(param_name, tmp) || tmp.size() != 3)
      return false;
    val = ignition::math::Vector3d(tmp[0], tmp[1], tmp[2]);
    return true;
  }
  else
    return false;
}
}
