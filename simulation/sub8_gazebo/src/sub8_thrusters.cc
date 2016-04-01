#include "sub8_gazebo/sub8_buoyancy.hpp"
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo {
class ThrusterPlugin : public ModelPlugin {
 public:
  ThrusterPlugin() : ModelPlugin() {}

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    GZ_ASSERT(_model != NULL, "Received NULL model pointer");
    this->model = _model;
    physics::WorldPtr world = _model->GetWorld();
    GZ_ASSERT(world != NULL, "Model is in a NULL world");
    this->physicsEngine = world->GetPhysicsEngine();
    GZ_ASSERT(this->physicsEngine != NULL, "Physics engine was NULL");

    GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");
    this->sdf = _sdf;

    // Make sure the ROS node for Gazebo has already been initialized
    GZ_ASSERT(!ros::isInitialized(), "ROS not initialized");
  }
};
GZ_REGISTER_WORLD_PLUGIN(ThrusterPlugin)
}
