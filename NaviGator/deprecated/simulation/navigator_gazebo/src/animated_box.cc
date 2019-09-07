//#include <QDebug>

#include "animated_box.h"

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(ModelPush)

void ModelPush::Load(gazebo::rendering::VisualPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  GZ_ASSERT(_parent != nullptr, "Received NULL model pointer");
  ros::Time::init();

  this->model = _parent;
  // Listen to the update event. This event is broadcast pre-render update???
  this->updateConnection = gazebo::event::Events::ConnectPreRender(boost::bind(&ModelPush::OnUpdate, this));
  begin_time = ros::Time::now();
}

// Called by the world update start event
void ModelPush::OnUpdate()
{
  if (this->model == nullptr)
  {
    return;
  }
  ros::Duration secs = ros::Time::now() - begin_time;
  gazebo::common::Color c = prev_color;

  if (count != 0 && secs.toSec() > .9)
  {
    if (count == 1)
    {
      c = green;
    }
    if (count == 2)
    {
      c = blue;
    }
    if (count == 3)
    {
      c = red;
    }
    begin_time = ros::Time::now();
    count = (count + 1) % 4;
  }

  else if (secs.toSec() > 1.9)
  {
    c = red;
    begin_time = ros::Time::now();
    count = (count + 1) % 4;
  }

  prev_color = c;

  this->model->SetAmbient(c);
  this->model->SetDiffuse(c);
}
