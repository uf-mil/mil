#ifndef ANIMATED_BOX_H
#define ANIMATED_BOX_H

#include <boost/bind.hpp>
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/rendering/Visual.hh"
#include "ros/ros.h"

class ModelPush : public gazebo::VisualPlugin
{
public:
  void Load(gazebo::rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/);
  // Called by the world update start event
  void OnUpdate();

private:
  // Pointer to the model
  gazebo::rendering::VisualPtr model = nullptr;
  // Pointer to the update event connection
  gazebo::event::ConnectionPtr updateConnection = nullptr;
  ros::Time begin_time;
  int count = 0;
  gazebo::common::Color prev_color;
  gazebo::common::Color red = gazebo::common::Color(1.0, 0.0, 0.0);
  gazebo::common::Color green = gazebo::common::Color(0.0, 1.0, 0.0);
  gazebo::common::Color blue = gazebo::common::Color(0.0, 0.0, 1.0);
  gazebo::common::Color black = gazebo::common::Color(0.0, 0.0, 0.0);
};

#endif  // ANIMATED_BOX_H
