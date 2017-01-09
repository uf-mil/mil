#include <broadcaster.hpp>

using namespace std;

AlarmBroadcaster::AlarmBroadcaster(ros::NodeHandle &nh, AlarmProxy* alarm)
{
  this->nh = nh;
  this->alarm = alarm;
  __set_alarm = this->nh.serviceClient<ros_alarms::AlarmSet>("/alarm/set");
  ROS_INFO((string("Created alarm broadcaster for alarm: ") + alarm->alarm_name).c_str());
}

bool AlarmBroadcaster::publish()
{
  ros_alarms::Alarm a = AlarmProxy(alarm->alarm_name, alarm->raised, alarm->node_name, alarm->problem_description,
                                   alarm->json_parameters, alarm->severity).as_msg();
  ros_alarms::AlarmSet srv;
  srv.request.alarm = a;
  return __set_alarm.call(srv);
}

