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
  msg.objects.reserve(objects_.size());
  for (auto& pair : objects_)
  {
    msg.objects.push_back(pair.second.as_msg());
  }
  return msg;
}

uint ObjectMap::add_object(point_cloud_ptr const& pc, KdTreePtr const& search_tree)
{
  auto id = highest_id_++;
  objects_.insert({id, Object(pc, id, search_tree) });
  return id;
}

ObjectMap::Iterator ObjectMap::erase_object(Iterator const& it)
{
  just_removed_.push_back((*it).first);
  return objects_.erase(it);
}

bool ObjectMap::DatabaseQuery(mil_msgs::ObjectDBQuery::Request& req, mil_msgs::ObjectDBQuery::Response& res)
{
  // Handle self classification command
  if (req.cmd != "")
  {
    if (req.cmd == "delete")
    {
      int id = -1;
      try
      {
        id = std::stoi(req.name);
      }
      catch (std::invalid_argument const& err)
      {
        res.found = false;
        return true;
      }

      for (auto pair = objects_.begin(); pair != objects_.end(); ++pair)
      {
        if ((*pair).first == id)
        {
          erase_object(pair);
          res.found = true;
          return true;
        }
      }

      res.found = false;
      return true;
    }

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
      it->second.set_classification(cmd);
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
    if (pair.second.as_msg().labeled_classification == req.name)
    {
      res.found = true;
      res.objects.emplace_back(pair.second.as_msg());
    }
  }

  return true;
}

}  // namespace pcodar
