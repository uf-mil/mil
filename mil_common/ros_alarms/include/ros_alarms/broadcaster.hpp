#pragma once

#include <ros/ros.h>
#include <ros_alarms_msgs/Alarm.h>
#include <ros_alarms_msgs/AlarmSet.h>

#include <ros_alarms/alarm_proxy.hpp>
#include <string>

namespace ros_alarms
{
/**
 * Facilitates operations on an alarm, such as raising or clearing an alarm.
 */
class AlarmBroadcaster
{
public:
  // Construct a broadcaster w/ an internally controlled (default) or externally controlled
  // (optional ptr argument) AlarmProxy
  /**
   * Constructs a broadcaster with an internally controlled (default) or externally controlled AlarmProxy.
   *
   * @param nh The node handle to associate with the broadcaster.
   * @param alarm The alarm proxy.
   */
  AlarmBroadcaster(ros::NodeHandle& nh, AlarmProxy* alarm = nullptr);

  /**
   * Clears the alarm, and publishes the change to the server.
   */
  void clear()
  {
    __alarm_ptr->raised = false;
    publish();
  }

  /**
   * Raises the alarm, and publishes the change to the server.
   */
  void raise()
  {
    __alarm_ptr->raised = true;
    publish();
  }

  /**
   * Updates the severity associated with an alarm, and raises the alarm. This
   * function should not be used to update the alarm severity to zero.
   *
   * @param sev The new severity.
   */
  void updateSeverity(int sev)
  {
    __alarm_ptr->severity = sev;
    raise();
  }

  /**
   * Publishes current state of the AlarmProxy to the server. If the publication encountered an error, a ROS warning is
   * sent alongside returning ``false``.
   *
   * @return The success of the publication.
   */
  bool publish();

  // Handle to update the AlarmProxy (if this is an externally managed AlPxy, then modifying
  // could potentially have side effects for other broadcasters that may use it. Exert Caution!)
  /**
   * Returns the alarm proxy associated with the broadcaster.
   *
   * Warning: If the class uses an externally modified alarm proxy, then modifying this class could have reprocussions
   * for other services which rely on that alarm proxy.
   *
   * @return The proxy.
   */
  AlarmProxy& getAlarm()
  {
    return *__alarm_ptr;
  }

private:
  ros::NodeHandle __nh;
  AlarmProxy* __alarm_ptr;   // All code should refer to the proxy via ptr
  AlarmProxy __alarm_proxy;  // This allows internal management of the alarm proxy
  ros::ServiceClient __set_alarm;
};

}  // namespace ros_alarms
