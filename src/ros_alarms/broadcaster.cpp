#include <ros_alarms/broadcaster.hpp>

namespace ros_alarms
{

using namespace std;

AlarmBroadcaster::AlarmBroadcaster(ros::NodeHandle &nh, AlarmProxy* alarm)
{
  __nh = nh;
  __alarm_ptr = alarm;
  __set_alarm = __nh.serviceClient<ros_alarms::AlarmSet>("/alarm/set");
  ROS_INFO((string("Created alarm broadcaster for alarm: ") + __alarm_ptr->alarm_name).c_str());
}

bool AlarmBroadcaster::publish()
{
  ros_alarms::Alarm a = AlarmProxy(__alarm_ptr->alarm_name, __alarm_ptr->raised, __alarm_ptr->node_name, __alarm_ptr->problem_description,
                                   __alarm_ptr->json_parameters, __alarm_ptr->severity).as_msg();
  ros_alarms::AlarmSet srv;
  srv.request.alarm = a;
  return __set_alarm.call(srv);
}

}  // namespace ros_alarms
