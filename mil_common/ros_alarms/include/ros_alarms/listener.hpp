/**
 * Author: David Soto
 * Date: Jan 16, 2017
 */
#pragma once

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros_alarms/Alarm.h>
#include <ros_alarms/AlarmGet.h>

#include <functional>
#include <ros_alarms/alarm_proxy.hpp>
#include <string>

namespace ros_alarms
{
/**
 * A callback associated with the Alarm Listener class. Helps to manage callbacks
 * by regulating their execution based on the current severity level and desired calling
 * behavior.
 */
template <typename callable_t = std::function<void(AlarmProxy)>>
struct ListenerCb
{
  /**
   * An enumerator representing when to run the callback.
   */
  enum class CallScenario
  {
    /**
     * Run the callback only if the alarm has been raised.
     */
    raise,

    /**
     * Run the callback only if the alarm has been cleared.
     */
    clear,

    /**
     * Run the callback under any condition.
     */
    always
  };

  /**
   * The specific callback function.
   */
  callable_t cb_func;  // object needs to have a call operator

  /**
   * The highest severity under which the callback can be called.
   */
  int severity_hi = 5;  // highest priority

  /**
   * The lowest severity under which the callback can be called.
   */
  int severity_lo = 0;  // lowest priority

  /**
   * The scenario under which the callback is executed.
   */
  CallScenario call_scenario = CallScenario::always;

  /**
   * Checks the severity of the alarm to determine whether the callback should
   * be executed. The severity should be between the lowest and highest severity
   * levels.
   *
   * @return Whether to execute the callback based on severity.
   */
  bool severity_check(int severity)
  {
    return severity <= severity_hi && severity >= severity_lo;
  }

  /**
   * Overloads the ``()`` operator to execute the callback function only if the
   * specified conditions are met.
   */
  void operator()(ros_alarms::Alarm msg)
  {
    // Only call if alarm status matches the call_scenario
    bool call_scenario_match = call_scenario == CallScenario::always ||
                               (call_scenario == CallScenario::raise && msg.raised) ||
                               (call_scenario == CallScenario::clear && !msg.raised);
    bool needs_call =
        call_scenario_match && (call_scenario == CallScenario::raise ? severity_check(msg.severity) : true);

    if (needs_call)
    {
      AlarmProxy alarm{ msg };
      cb_func(alarm);
    }
  }
};

/**
 * Listens to a specific ROS alarm using AlarmProxy.
 */
template <typename callable_t = std::function<void(AlarmProxy)>>
class AlarmListener
{
  using CallScenario = typename ListenerCb<callable_t>::CallScenario;

public:
  /**
   * Default constructor.
   *
   * @param nh The node handle.
   * @param alarm_name The name of the alarm connected to the listener.
   */
  AlarmListener(ros::NodeHandle &nh, std::string alarm_name);

  /**
   * Return the status of the listener.
   *
   * @return Whether the listener is operating OK.
   */
  bool ok() const
  {
    return __ok;
  }

  /**
   * Get the number of publishers connected to the ROS node.
   *
   * @return The number of connections.
   */
  int getNumConnections()
  {
    return __update_subscriber.getNumPublishers();
  }

  // Returns true if a connection was detected before timing out, else false
  /**
   * Waits for a connection to be established, and returns true if a connection
   * was established.
   *
   * @param timeout The amount of time (in seconds) before the function exits and
   * returns false. Default time is -1 seconds, in which the function will never
   * return false.
   *
   * @return Whether a connection was established.
   */
  bool waitForConnection(ros::Duration timeout = { -1.0 });

  /**
   * Starts the async spinner.
   */
  void start()
  {
    __async_spinner.start();
  }

  /**
   * Stops the async spinner.
   */
  void stop()
  {
    __async_spinner.stop();
  }

  // Functions that return the status of the alarm at time of last update

  /**
   * Returns a cached value of whether the alarm has been raised. Avoids querying
   * the server by using the cache.
   *
   * @return Whether the alarm has been raised, according to the cache.
   */
  bool isRaised() const
  {
    return __last_alarm.raised;
  }

  /**
   * Returns a cached value of whether the alarm is cleared. Does not query the alarm
   * server.
   *
   * @return Whether the alarm is cleared, according to the cache.
   */
  bool isCleared() const
  {
    return !isRaised();
  }

  // Functions that query the server before returning the latest status of the alarm
  /**
   * Queries the alarm server to see if an alarm with this name has been raised.
   *
   * @return Whether the found alarm was raised.
   */
  bool queryRaised()
  {
    getAlarm();
    return __last_alarm.raised;
  }

  /**
   * Queries the alarm server to see if this alarm has been cleared.
   *
   * @return Whether this alarm was cleared, according to the alarm server.
   */
  bool queryCleared()
  {
    getAlarm();
    return !isRaised();
  }

  /**
   * Queries the alarm server for an alarm of the given name.
   *
   * @return The found alarm with the given name.
   */
  AlarmProxy getAlarm();

  /**
   * Returns the most recently cached alarm returned from the alarm server. Avoids
   * a query by using a cache.
   *
   * @return The most recently cached alarm.
   */
  AlarmProxy getCachedAlarm()
  {
    return __last_alarm;
  }

  /**
   * Registers a callback to be invoked on both raise and clear.
   *
   * @param cb The callback function.
   */
  void addCb(callable_t cb);

  /**
   * Registers a callback to be invoked for any severity level.
   *
   * @param cb The callback function.
   */
  void addRaiseCb(callable_t cb);

  /**
   * Registers a callback to be invoked for a single severity level.
   *
   * @param cb The callback function.
   * @param severity The severity level to execute the callback for.
   */
  void addRaiseCb(callable_t cb, int severity);

  /**
   * Registers a callback to be invoked for a range of severity levels.
   *
   * @param cb The callback function.
   * @param severity_lo The lowest severity level to execute the callback for.
   * @param severity_hi The highest severity level to execute the callback for.
   */
  void addRaiseCb(callable_t cb, int severity_lo, int severity_hi);

  /**
   * Registers a callback to be invoked when an alarm is cleared.
   *
   * @param cb The callback function.
   */
  void addClearCb(callable_t cb);

  /**
   * Returns the last time the alarm was updated.
   *
   * @return The most recent update time.
   */
  ros::Time getLastUpdateTime() const
  {
    return __last_update;
  }

  /**
   * Returns the amount of time since the alarm was updated.
   *
   * @return The length of time since the alarm was updated.
   */
  ros::Duration getTimeSinceUpdate() const
  {
    return ros::Time::now() - __last_update;
  }

  /**
   * Returns true if a new update occurs before the function times out. If the function
   * times out, returns false.
   *
   * @return Whether the update occurred.
   */
  bool waitForUpdate(ros::Duration timeout = ros::Duration(-1.0)) const;

  /**
   * Clears all callback associated with the listener.
   */
  void clearCallbacks()
  {
    __callbacks.clear();
  }

private:
  bool __ok = true;  // listener fail-bit
  ros::NodeHandle __nh;
  std::string __alarm_name;
  ros::ServiceClient __get_alarm;
  ros::CallbackQueue __cb_queue;
  ros::AsyncSpinner __async_spinner;
  ros::Subscriber __update_subscriber;
  std::vector<ListenerCb<callable_t>> __callbacks;
  AlarmProxy __last_alarm;
  ros::Time __last_update{ 0, 0 };
  std::string __object_name;
  void __addCb(callable_t cb, int severity_lo, int severity_hi, CallScenario call_scenario);
  void __alarmUpdate(ros_alarms::Alarm);
};

template <typename callable_t>
AlarmListener<callable_t>::AlarmListener(ros::NodeHandle &nh, std::string alarm_name)
try : __nh(nh),
      __alarm_name(alarm_name),
      __get_alarm(__nh.serviceClient<ros_alarms::AlarmGet>("/alarm/get")),
      __async_spinner(1, &__cb_queue)

{
  std::stringstream obj_name;  // For better error msgs
  obj_name << "AlarmListener[alarm_name=" << __alarm_name << ", node_name=" << ros::this_node::getName() << "]";
  __object_name = obj_name.str();

  // Use owned callback queue
  __nh.setCallbackQueue(&__cb_queue);
  __update_subscriber = __nh.subscribe("/alarm/updates", 1000, &AlarmListener::__alarmUpdate, this);

  // Service to query alarm server
  bool service_exists = __get_alarm.waitForExistence(ros::Duration(1.0));
  if (!service_exists)
  {
    __ok = false;
    std::string msg = __object_name;
    msg += ": timed out waiting for service /alarm/get";
    ROS_WARN("%s", msg.c_str());
  }

  try  // Initial server query so stored AlarmProxy is initialized
  {
    getAlarm();
  }
  catch (std::exception &e)
  {
    std::stringstream msg;
    msg << __object_name << " getAlarm() threw an exception: " << e.what();
    ROS_ERROR("%s", msg.str().c_str());
  }
}
catch (std::exception &e)
{
  __ok = false;
  std::stringstream msg;
  msg << __object_name << "  Constructor threw an exception: " << e.what();
  ROS_ERROR("%s", msg.str().c_str());
}

/**
 * Waits for a connection to connect to the subscriber as a publisher.
 *
 * @param timeout The amount of time before the function returns ``false``.
 *
 * @return Whether a connection has been established.
 */
template <typename callable_t>
bool AlarmListener<callable_t>::waitForConnection(ros::Duration timeout)
{
  ros::Time start = ros::Time::now();
  while (ros::Time::now() - start < timeout)
  {
    if (this->getNumConnections() > 0)
      return true;
    ros::Duration(1E-3).sleep();
  }
  return false;
}

template <typename callable_t>
AlarmProxy AlarmListener<callable_t>::getAlarm()
{
  // Create Query msg
  ros_alarms::AlarmGet alarm_query;
  alarm_query.request.alarm_name = __alarm_name;

  // Query alarm server
  if (!__get_alarm.call(alarm_query))
    ROS_INFO("%s: %s", __object_name.c_str(), "Alarm server query was unsuccessful.");

  // Update internal alarm data
  __last_update = alarm_query.response.header.stamp;
  __last_alarm = AlarmProxy(alarm_query.response.alarm);
  return __last_alarm;
}

template <typename callable_t>
void AlarmListener<callable_t>::addCb(callable_t cb)
{
  __addCb(cb, 0, 5, CallScenario::always);
}

template <typename callable_t>
void AlarmListener<callable_t>::addRaiseCb(callable_t cb)
{
  __addCb(cb, 0, 5, CallScenario::raise);
}

template <typename callable_t>
void AlarmListener<callable_t>::addRaiseCb(callable_t cb, int severity)
{
  __addCb(cb, severity, severity, CallScenario::raise);
}

template <typename callable_t>
void AlarmListener<callable_t>::addRaiseCb(callable_t cb, int severity_lo, int severity_hi)
{
  __addCb(cb, severity_lo, severity_hi, CallScenario::raise);
}

template <typename callable_t>
void AlarmListener<callable_t>::addClearCb(callable_t cb)
{
  __addCb(cb, 0, 5, CallScenario::clear);
}

template <typename callable_t>
bool AlarmListener<callable_t>::waitForUpdate(ros::Duration timeout) const
{
  auto start = ros::Time::now();
  auto old_update_time = getLastUpdateTime();
  while (ros::Time::now() - start < timeout)
  {
    if (getLastUpdateTime() != old_update_time)
      return true;
  }
  return false;
}
template <typename callable_t>
void AlarmListener<callable_t>::__addCb(callable_t cb, int severity_lo, int severity_hi, CallScenario call_scenario)
{
  ListenerCb<callable_t> l_cb;
  l_cb.cb_func = cb;
  l_cb.severity_lo = severity_lo;
  l_cb.severity_hi = severity_hi;
  l_cb.call_scenario = call_scenario;

  __callbacks.push_back(l_cb);
}

template <typename callable_t>
void AlarmListener<callable_t>::__alarmUpdate(ros_alarms::Alarm alarm_msg)
{
  if (alarm_msg.alarm_name == __alarm_name)
  {
    // Update internal alarm data
    __last_alarm = alarm_msg;
    __last_update = ros::Time::now();

    // Invoke callbacks if necessary
    for (auto &cb : __callbacks)
      try
      {
        cb(alarm_msg);
      }
      catch (std::exception &e)
      {
        std::stringstream msg;
        msg << __object_name << ": callback threw an exception: " << e.what();
        ROS_ERROR("%s", msg.str().c_str());
      }
  }
}

}  // namespace ros_alarms
