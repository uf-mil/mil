#ifndef ANIMATED_BOX_H
#define ANIMATED_BOX_H

#include <boost/bind.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/common/common.hh"
#include <ctime>

namespace gazebo
{
class ModelPush : public VisualPlugin
{
public:
    void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/);
    // Called by the world update start event
    void OnUpdate();

private:
    // Pointer to the model
    rendering::VisualPtr model = NULL;
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection = NULL;
    clock_t begin_time;
    int count = 0;
    common::Color prev_color;
};
}

#endif // ANIMATED_BOX_H