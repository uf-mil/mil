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
    ROS_DEBUG("MAPPING %s to %s", it.first.c_str(), it.second.c_str());
  }
  UpdateEntities();
  timer_ = nh_.createTimer(ros::Duration(5.0), std::bind(&PCODARGazebo::TimerCb, this, std::placeholders::_1));
}

void PCODARGazebo::TimerCb(const ros::TimerEvent&)
{
  pcodar_->UpdateObjects();
}

void PCODARGazebo::UpdateEntities()
{
  for (auto model : world_->Models())
  {
    UpdateModel(model);
  }
}

void PCODARGazebo::UpdateModel(gazebo::physics::ModelPtr _model)
{
  UpdateEntity(_model);

  // Recurse into nested models
  for (auto model : _model->NestedModels())
  {
    UpdateModel(model);
  }

  // Recurse into links
  for (auto link : _model->GetLinks())
  {
    UpdateEntity(link);
  }
}

void PCODARGazebo::UpdateEntity(gazebo::physics::EntityPtr _entity)
{
  ROS_DEBUG("Entity %s", _entity->GetName().c_str());

  // Find this entity in the name map
  auto it = name_map_.find(_entity->GetName());

  // Ignore if not in the map
  if (it == name_map_.end())
    return;

  int id = _entity->GetId();

  // Create an object message for this entity
  pcodar::Object object = pcodar::Object(pcodar::point_cloud());
  object.msg_.labeled_classification = (*it).second;

  // Get pose and bounding box for object
  ignition::math::Pose3d pose = _entity->WorldPose();
  ignition::math::Box box = _entity->BoundingBox();
  // Move pcdoar pose origin to center of bounding box
  pose.Set(box.Center(), pose.Rot());

  // Convert pose and box to ROS messages
  GazeboPoseToRosMsg(pose, object.msg_.pose);
  GazeboVectorToRosMsg(box.Size(), object.msg_.scale);

  // Add or update object
  auto existing_object = pcodar_->objects_->objects_.find(id);
  if (existing_object == pcodar_->objects_->objects_.end())
    pcodar_->objects_->objects_.insert({ id, object });
  else
    (*existing_object).second = object;
}

void PCODARGazebo::GazeboPoseToRosMsg(ignition::math::Pose3d const& in, geometry_msgs::Pose& out)
{
  out.position.x = in.Pos().X();
  out.position.y = in.Pos().Y();
  out.position.z = in.Pos().Z();

  out.orientation.x = in.Rot().X();
  out.orientation.y = in.Rot().Y();
  out.orientation.z = in.Rot().Z();
  out.orientation.w = in.Rot().W();
}

void PCODARGazebo::GazeboVectorToRosMsg(ignition::math::Vector3d const& in, geometry_msgs::Vector3& out)
{
  out.x = in.X();
  out.y = in.Y();
  out.z = in.Z();
}

GZ_REGISTER_WORLD_PLUGIN(PCODARGazebo)
}
