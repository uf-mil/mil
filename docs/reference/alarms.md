# ROS Alarms Part 1

In the realm of building dependable control systems, the importance of error detection and effective error-handling mechanisms cannot be overstated. Within this context, MIL presents a robust solution in the form of a live alarm system. This alarm system operates discreetly in the background of both the robot's mission and driver codebases, ready to be activated upon the emergence of errors. Notably, the alarm code doesn't solely serve to identify and address errors; it can also adeptly manage changes or updates that extend beyond error scenarios.

## ROS Alarms: A Service-Oriented Architecture

The architecture of ROS alarms distinguishes itself by employing a service-oriented model rather than the usual topic-based approach. In ROS, Services act as the conduits for interaction between nodes, functioning in a request-response manner. While ROS topics enable asynchronous data exchange, services facilitate nodes in seeking specific actions or information from other nodes, awaiting a subsequent response before proceeding. This method of waiting before proceeding is known as a synchronous data exchange. This proves especially valuable in tasks that require direct engagement, such as data retrieval or computations.
If you are not familiar with ROS Services it is highly recommended that you take a look at [our page on ROS services](./services.md).

## Alarm System Logic

The alarm system's functionality is more intricate than that of a typical ROS service, which usually manages operations of base types (ints, strings, etc.). In this scenario, the alarm's service server is engineered to manage the tasks of updating, querying, and processing an alarm object. ROS alarms encompass two distinct types of clients: the alarm broadcaster and the alarm listener. The broadcaster initializes and triggers alarms in response to errors or changes, while the listener monitors the broadcaster's activity and activates designated a callback function when alarms are raised. The callback function should handle the error or change appropriately. 

To fully understand the logic behind the alarm system code, refer to this folder: [https://github.com/uf-mil/mil/tree/master/mil_common/ros_alarms](https://github.com/uf-mil/mil/tree/master/mil_common/ros_alarms)

To successfully leverage alarms, the initialization of both the broadcaster and listener is needed. The listener should be configured to execute a predefined callback function, addressing errors or changes detected by the broadcaster. Within your codebase, error detection and alarm-raising procedures should be integrated. If orchestrated correctly, the callback function will be automatically invoked, underscoring successful error mitigation.

Note that there are several special properties that can be attached to your alarm. Here are a couple of examples: 
* When you raise an alarm you can assign a severity level to the alarm [0, 5]. 
* You can attach multiple callback functions to the alarm.
  * **This is where severity comes into play!** By specifying the required severity level that is needed to execute the callback when initializing the function, you can choose which callbacks are executed when the alarm is raised.
  * You can also specify a range of severity levels that the alarm would need to execute a given callback.

Here is a line-by-line breakdown of an example alarm implementation: 

```python
    ab = AlarmBroadcaster("test_alarm")
    al = AlarmListener("test_alarm")
    ab.clear_alarm()
    rospy.sleep(0.1)
```
This is how you would initialize the alarm broadcaster and listener. Here they make sure to clear any previous alarm data in the broadcaster. 

```python
    al.add_callback(cb1)
```
Make sure to establish the callback function that should be executed once the alarm is activated. 

```python
    ab.raise_alarm()
    rospy.sleep(0.1)
    assert al.is_raised()
    assert cb1_ran
```
When the alarm is sounded via the `raise_alarm()` function, the callback will be executed automatically. 

```python
    al.clear_callbacks()

    al.add_callback(cb1, severity_required=2)
    al.add_callback(cb2, call_when_raised=False)

    rospy.loginfo("Severity Range Test 1")
    ab.raise_alarm(severity=4)
    rospy.sleep(0.1)
    assert not cb1_ran
    assert cb2_ran
    cb2_ran = False

    rospy.loginfo("Severity Range Test 2")
    ab.raise_alarm(severity=1)
    rospy.sleep(0.1)
    assert cb1_ran
    assert not cb2_ran
    cb1_ran = False
```
Note that you can also attach some special properties to your alarm. For instance, you can attach multiple callback functions to the alarm. You can also configure whether the callback function should be automatically executed when the alarm is raised or whether it should be executed manually. Finally, you can assign a severity level to the alarm which can tell the alarm code which callback functions should be run. 

For a practical example of this workflow, visit: [https://github.com/uf-mil/mil/blob/master/mil_common/ros_alarms/test/rospy/callback_test.py](https://github.com/uf-mil/mil/blob/master/mil_common/ros_alarms/test/rospy/callback_test.py)

## Applications and Context

The applications of ROS alarms span various contexts, with one notable application residing in the control of the submersible vehicle's thrust and killboard. The thrust and killboard, responsible for the sub's electronic operations, is integrally associated with ROS alarms. Upon the board's activation or deactivation (hard or soft kill), alarms are invoked to apprise users of these changes. The listener's callback function comes into play, ensuring that alarms are updated in alignment with the board's current state. This process triggered each time the board is deactivated, creates a system whereby users are continually informed about the board's status changes – essentially manifesting a dynamic live alarm system.

To delve into the implementation, visit: [https://github.com/uf-mil/mil/blob/master/SubjuGator/drivers/sub8_thrust_and_kill_board/sub8_thrust_and_kill_board/handle.py](https://github.com/uf-mil/mil/blob/master/SubjuGator/drivers/sub8_thrust_and_kill_board/sub8_thrust_and_kill_board/handle.py)

## Part 2
Tale a look at all of the methods ros_alarms offers here: [https://uf-mil.github.io/docs/reference/alarms.html?highlight=alarms#module-ros_alarms](https://uf-mil.github.io/docs/reference/alarms.html?highlight=alarms#module-ros_alarms)
