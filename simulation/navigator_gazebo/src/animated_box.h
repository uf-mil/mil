#ifndef ANIMATED_BOX_H
#define ANIMATED_BOX_H

#include <boost/bind.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/common/common.hh"
#include <ctime>

class ModelPush : public gazebo::VisualPlugin {
public:
  void Load(gazebo::rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/);
  // Called by the world update start event
  void OnUpdate();

private:
  // Pointer to the model
  gazebo::rendering::VisualPtr model = nullptr;
  // Pointer to the update event connection
  gazebo::event::ConnectionPtr updateConnection = nullptr;
  clock_t begin_time;
  int count = 0;
  gazebo::common::Color prev_color;
};

#endif // ANIMATED_BOX_H
