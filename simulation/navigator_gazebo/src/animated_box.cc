//#include <QDebug>

#include "animated_box.h"

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(ModelPush)

void ModelPush::Load(gazebo::rendering::VisualPtr _parent,
                     sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");

  this->model = _parent;
  // Listen to the update event. This event is broadcast pre-render update???
  this->updateConnection = gazebo::event::Events::ConnectPreRender(
      boost::bind(&ModelPush::OnUpdate, this));
  begin_time = clock();
}

// Called by the world update start event
void ModelPush::OnUpdate() {
  if (this->model == nullptr) {
    return;
  }
  float secs = float(clock() - begin_time) / CLOCKS_PER_SEC;
  gazebo::common::Color c = prev_color;

  if (count != 0 && secs > .9) {
    if (count == 1) {
      c = gazebo::common::Color(0.0, 1.0, 0.0);
    }
    if (count == 2) {
      c = gazebo::common::Color(0.0, 0.0, 1.0);
    }
    if (count == 3) {
      c = gazebo::common::Color(0.0, 0.0, 0.0);
    }
    begin_time = clock();
    count = (count + 1) % 4;

  }

  else if (secs > 1.9) {
    c = gazebo::common::Color(1.0, 0.0, 0.0);
    begin_time = clock();
    count = (count + 1) % 4;
  }

  prev_color = c;

  this->model->SetAmbient(c);
  this->model->SetDiffuse(c);
}
