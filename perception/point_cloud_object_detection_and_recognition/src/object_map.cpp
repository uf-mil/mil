#include <point_cloud_object_detection_and_recognition/object_map.hpp>
#include <mil_msgs/PerceptionObject.h>

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
  objects_.insert({highest_id_++, Object(pc)});
}

} // namespace pcodar
