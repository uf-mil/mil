#ifndef MIL_GAZEBO_UTILS_H
#define MIL_GAZEBO_UTILS_H

#include <geometry_msgs/Vector3.h>
#include <gazebo/common/common.hh>

namespace mil_gazebo
{
void Convert(ignition::math::Vector3d const& _in, geometry_msgs::Vector3& _out);

void Convert(gazebo::common::Time const& _in, ros::Time& _out);

double NoiseCovariance(gazebo::sensors::Noise const& _noise);

/**
 * Determines weither a tag is set in sdf or has the ros paramter refered
 * to with the value of that tag + "_param"
 *
 * @param _sdf: Pointer to the parent sdf element to find the tag
 * @param _tag: The name of the tag to search for
 * @return 0 if the tag is present in the sdf
 *         1 if tag+"_param" is present in the sdf and this ROS param exists
 *         -1 if neither of the above cases is true
 */
int TagInSDFOrRosParam(sdf::ElementPtr _sdf, std::string const& tag);

/**
 *
 */
bool GetFromSDFOrRosParam(sdf::ElementPtr _sdf, std::string const& tag, double& val);

/**
 *
 */
bool GetFromSDFOrRosParam(sdf::ElementPtr _sdf, std::string const& tag, ignition::math::Vector3d& val);
}

#endif  // MIL_GAZEBO_UTILS_H
