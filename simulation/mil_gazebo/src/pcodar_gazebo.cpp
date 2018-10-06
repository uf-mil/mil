#include <mil_gazebo/pcodar_gazebo.hpp>
#include <mil_msgs/PerceptionObject.h>

namespace mil_gazebo
{


void PCODARGazebo::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  world_ = _world;
  nh_ = ros::NodeHandle("pcodar");
  nh_.getParam("/gazebo/name_map", name_map_);
  for (auto it : name_map_)
  {
    ROS_WARN("MAPPING %s to %s", it.first.c_str(), it.second.c_str());
  }
  server_ = nh_.advertiseService(std::string("/database/requests"), &PCODARGazebo::DatabaseQuery, this);
}

bool PCODARGazebo::DatabaseQuery(mil_msgs::ObjectDBQuery::Request &req, mil_msgs::ObjectDBQuery::Response& res)
{
  for (auto model: world_->GetModels())
  {
    AddDatabaseObject(req, res, model);
  }
  return true;
}

void PCODARGazebo::AddDatabaseObject(mil_msgs::ObjectDBQuery::Request &req, mil_msgs::ObjectDBQuery::Response& res, gazebo::physics::ModelPtr _model)
{
  ROS_WARN("OBJECT %s", _model->GetName().c_str());
  auto it = name_map_.find(_model->GetName());
  if (it != name_map_.end())
  {
    res.found = true;
    // TODO: only add if matches query string
    mil_msgs::PerceptionObject object;
    object.classification = (*it).second;
    GazeboPoseToRosMsg(_model->GetWorldPose(), object.pose);
    res.objects.push_back(object);
  }

  // Recurse into nested models
  for (auto model: _model->NestedModels())
  {
    AddDatabaseObject(req, res, model);
  }

  // Recurse into links
  for (auto link: _model->GetLinks())
  {
    AddDatabaseLink(req, res, link);
  }
}

void PCODARGazebo::AddDatabaseLink(mil_msgs::ObjectDBQuery::Request &req, mil_msgs::ObjectDBQuery::Response& res, gazebo::physics::LinkPtr _link)
{
  ROS_WARN("LINK %s", _link->GetName().c_str());

  auto it = name_map_.find(_link->GetName());
  if (it == name_map_.end()) return;
  res.found = true;
  // TODO: only add if matches query string
  mil_msgs::PerceptionObject object;
  object.classification = (*it).second;
  GazeboPoseToRosMsg(_link->GetWorldPose(), object.pose);
  res.objects.push_back(object);
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

GZ_REGISTER_WORLD_PLUGIN(PCODARGazebo)

}


