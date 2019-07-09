
#ifndef MIL_DRAG_GAZEBO_HPP
#define MIL_DRAG_GAZEBO_HPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

namespace mil_gazebo
{
/// \brief Plugin to simulate a DVL
class MilDragGazebo : public gazebo::ModelPlugin
{
  /// Constructor
public:
  MilDragGazebo();

  /// Destructor
public:
  ~MilDragGazebo();

public:
  void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf);

private:
  void OnUpdate(const gazebo::common::UpdateInfo& info);

  ignition::math::Pose3d offset_;
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::math::Vector3 linear_coeffs_;
  gazebo::math::Vector3 angular_coeffs_;
  gazebo::event::ConnectionPtr update_connection_;
};
}

#endif
