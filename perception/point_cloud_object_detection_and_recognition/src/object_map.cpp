#include <mil_msgs/PerceptionObject.h>
#include <point_cloud_object_detection_and_recognition/object_map.hpp>

namespace pcodar
{
ObjectMap::ObjectMap() : highest_id_(0)
{
}

mil_msgs::PerceptionObjectArray ObjectMap::to_msg()
{
  mil_msgs::PerceptionObjectArray msg;
  for (auto& pair : objects_)
  {
    pair.second.msg_.id = pair.first;
    msg.objects.push_back(pair.second.msg_);
  }
  return msg;
}

void ObjectMap::add_object(point_cloud pc)
{
  objects_.insert({ highest_id_++, Object(pc) });
}

bool ObjectMap::DatabaseQuery(mil_msgs::ObjectDBQuery::Request& req, mil_msgs::ObjectDBQuery::Response& res)
{
  // Handle self classification command
  if (req.cmd != "")
  {
    int pos = req.cmd.find_first_of("=");

    int id = -1;
    try
    {
      id = std::stoi(req.cmd.substr(0, pos));
    }
    catch (std::invalid_argument const& err)
    {
      res.found = false;
      return true;
    }
    std::string cmd = req.cmd.substr(pos + 1);
    auto it = objects_.find(id);
    if (objects_.end() == it)
    {
      res.found = false;
    }
    else
    {
      it->second.msg_.labeled_classification = cmd;
      res.found = true;
    }
    return true;
  }

  // Handle request all command
  if ("all" == req.name)
  {
    res.found = true;
    res.objects = to_msg().objects;
    return true;
  }

  // Handle request particular
  res.found = false;
  for (auto const& pair : objects_)
  {
    if (pair.second.msg_.classification == req.name || pair.second.msg_.labeled_classification == req.name)
    {
      res.found = true;
      res.objects.push_back(pair.second.msg_);
    }
  }

  return true;
}

}  // namespace pcodar
