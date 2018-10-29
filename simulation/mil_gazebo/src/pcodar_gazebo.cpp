#include <mil_msgs/PerceptionObject.h>
#include <mil_gazebo/pcodar_gazebo.hpp>

namespace mil_gazebo
{
void PCODARGazebo::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  world_ = _world;
  nh_ = ros::NodeHandle("/pcodar");
  nh_.getParam("/gazebo/name_map", name_map_);
  pcodar_.reset(new pcodar::NodeBase(nh_));
  pcodar_->initialize();
  for (auto it : name_map_)
  {
    ROS_WARN("MAPPING %s to %s", it.first.c_str(), it.second.c_str());
  }
  UpdateObjects();
  timer_ = nh_.createTimer(ros::Duration(5.0), std::bind(&PCODARGazebo::TimerCb, this, std::placeholders::_1));
}

void PCODARGazebo::TimerCb(const ros::TimerEvent&)
{
  pcodar_->UpdateObjects();
}

void PCODARGazebo::UpdateObjects()
{
  for (auto model : world_->GetModels())
  {
    UpdateObject(model);
  }
}

void PCODARGazebo::UpdateObject(gazebo::physics::ModelPtr _model)
{
  ROS_WARN("OBJECT %s", _model->GetName().c_str());
  auto it = name_map_.find(_model->GetName());
  if (it != name_map_.end())
  {
    int id = _model->GetId();
    pcodar::Object object = pcodar::Object(pcodar::point_cloud());
    object.msg_.classification = (*it).second;
    GazeboPoseToRosMsg(_model->GetWorldPose(), object.msg_.pose);
    GazeboVectorToRosMsg(_model->GetBoundingBox().GetSize(), object.msg_.scale);

    auto existing_object = pcodar_->objects_.objects_.find(id);
    if (existing_object == pcodar_->objects_.objects_.end())
      pcodar_->objects_.objects_.insert({ id, object });
    else
      (*existing_object).second = object;
  }

  // Recurse into nested models
  for (auto model : _model->NestedModels())
  {
    UpdateObject(model);
  }

  // Recurse into links
  for (auto link : _model->GetLinks())
  {
    UpdateLink(link);
  }
}

void PCODARGazebo::UpdateLink(gazebo::physics::LinkPtr _link)
{
  ROS_WARN("LINK %s", _link->GetName().c_str());

  auto it = name_map_.find(_link->GetName());
  if (it == name_map_.end())
    return;

  int id = _link->GetId();
  pcodar::Object object = pcodar::Object(pcodar::point_cloud());
  object.msg_.classification = (*it).second;
  GazeboPoseToRosMsg(_link->GetWorldPose(), object.msg_.pose);
  GazeboVectorToRosMsg(_link->GetBoundingBox().GetSize(), object.msg_.scale);

  auto existing_object = pcodar_->objects_.objects_.find(id);
  if (existing_object == pcodar_->objects_.objects_.end())
    pcodar_->objects_.objects_.insert({ id, object });
  else
    (*existing_object).second = object;
}

void PCODARGazebo::GazeboPoseToRosMsg(gazebo::math::Pose const& in, geometry_msgs::Pose& out)
{
  out.position.x = in.pos.x;
  out.position.y = in.pos.y;
  out.position.z = in.pos.z;

  out.orientation.x = in.rot.x;
  out.orientation.y = in.rot.y;
  out.orientation.z = in.rot.z;
  out.orientation.w = in.rot.w;
}

void PCODARGazebo::GazeboVectorToRosMsg(gazebo::math::Vector3 const& in, geometry_msgs::Vector3& out)
{
  out.x = in.x;
  out.y = in.y;
  out.z = in.z;
}

GZ_REGISTER_WORLD_PLUGIN(PCODARGazebo)
}
