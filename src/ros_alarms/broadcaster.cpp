#include <ros_alarms/broadcaster.hpp>

namespace ros_alarms
{

using namespace std;

AlarmBroadcaster::AlarmBroadcaster(ros::NodeHandle &nh, AlarmProxy* alarm)
: __nh(nh),
  __alarm_ptr(alarm),
  __alarm_proxy("uninitialized_alarm", false, "", "", 5),
  __set_alarm(__nh.serviceClient<ros_alarms::AlarmSet>("/alarm/set"))
{
  // Broadcaster should use the AlarmProxy allocated internally if the user did
  // not provide an external one to use
  if(__alarm_ptr == nullptr)
  {
    __alarm_ptr = &__alarm_proxy;
  }
  stringstream msg;
  msg << "Node " << __alarm_proxy.node_name << " created an AlarmBroadcaster for alarm: "
      << __alarm_ptr->str();
  ROS_INFO("%s", msg.str().c_str());
}

bool AlarmBroadcaster::publish()
{
  ros_alarms::Alarm a = AlarmProxy(__alarm_ptr->alarm_name, __alarm_ptr->raised, __alarm_ptr->node_name,
                                   __alarm_ptr->problem_description, __alarm_ptr->json_parameters,
                                   __alarm_ptr->severity).as_msg();
  ros_alarms::AlarmSet srv;
  srv.request.alarm = a;
  bool success = __set_alarm.call(srv);
  if(!success)
  {
    std::string msg = "Failed to register " + __alarm_ptr->str() + "with the alarm server";
    ROS_WARN("%s", msg.c_str());
  }
  return success;
}

}  // namespace ros_alarms
