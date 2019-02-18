#ifndef MIL_GAZEBO_UTILS_H
#define MIL_GAZEBO_UTILS_H

#include <geometry_msgs/Vector3.h>
#include <gazebo/common/common.hh>

namespace mil_gazebo
{

void Convert(ignition::math::Vector3d const& _in, geometry_msgs::Vector3& _out);

void Convert(gazebo::common::Time const& _in, ros::Time& _out);

double NoiseCovariance(gazebo::sensors::Noise const& _noise);

}

#endif // MIL_GAZEBO_UTILS_H
