## Alarm Structure

An alarm is be described by the following:

1. **action_required**: bool
1. **problem_description**: string
1. **severity**: int -> 0 is most severe
1. **problem_flag**: char -> indexes into table of possible system alarms maintained by the alarm node
1. **node_id**: string -> the node where the alarm originated from

## Alarm Severity

* 0 - FAILURE (fatal)
* 1 - CRITICAL (Usually fatal)
* 2 - WARNING (Non-fatal)
* 3 - DEFAULT (Nothing wrong)

## Alarm Node

The Sub8 alarm system has two key components. AlarmBroadcasters, which "raise" alarms, and Alarm Scenarios, which handle alarms. In addition to those, the alarm system relies on each node exposing some behavior like "kill", or "disable_thruster" so that the alarm system can *do* something with the alarm state.

## AlarmBroadcaster

Here is an example use of an AlarmBroadcaster

    from sub8_alarm import AlarmBroadcaster

    if __name__ == '__main__':
        alarm_broadcaster = AlarmBroadcaster()
        thruster_out_alarm = alarm_broadcaster.add_alarm(
            name='thruster_out',
            action_required=False,
            severity=3
        )

        # ...
        # things happen, and we detect a thruster failure
        # ...

        thruster_out_alarm.raise_alarm(
            problem_description='Thruster {} has failed'.format('BRV'),
            parameters={
                'thruster_name': 'BRV',
                'fault_info': {'bus_voltage': 45, 'fault': 0x20}
            }
        )

Let's break it down segment by segment


    alarm_broadcaster = AlarmBroadcaster()


First, we create an alarm_broadcaster object. This currently handles some publishing business behind the scenes. In the future, it may automatically generate and connect services, so that alarms can be published even as a node is going offline, using rospy shutdown hooks.


    thruster_out_alarm = alarm_broadcaster.add_alarm(
        name='thruster_out',
        action_required=True,
        severity=3
    )


Then we add an alarm to our broadcaster. This alarm is called "thruster_out", because we will raise it whenever a thruster goes out. We also specify two other parameters, `action_required`, which tells the Sub whether or not it needs to *immediately* do something or change some state. Another example of an alarm requiring action might be "new obstacle in field of view". An example of an alarm that might not require immediate action is "bus voltage is approaching safety limit".

The other parameter is severity. A lower severity numeber is more urgent. This is because we wanted to allow easy addition of lower importance alarm severities. When many alarms must be handled, they will be handled as a priority queue, and not a deque. The priority of each alarm in the queue will be the severity.


    thruster_out_alarm.raise_alarm(
        problem_description='Thruster {} has failed'.format('BRV'),
        parameters={
            'thruster_name': 'BRV',
            'fault_info': {'bus_voltage': 45, 'fault': 0x20}
        }
    )


Finally, we raise our alarm. We give the alarm a **human readable** `problem_description`. We also supply a non-necessarily-human-readable description. Note that we simply supply a dictionary (You could also supply just a list, or a string or int, or whatever you like). The parameters you supply are serialized to JSON, internally, and sent over the alarm topic. It is then deserialized on the other end.


## Alarm Scenarios

Alarm scenarios are the way we handle a particular alarm. This decouples response behavior from failure detection. Every alarm scenario (in sub8_alarm.alarms), is a class that must:

* Capitalize its first letter
* Supply an `alarm_name` attribute, that is the name of the alarm it should respond to
* Define a `handle` function, that takes (time_sent, parameters) as arguments. It will be automatically called when an alarm with name `$alarm_name` is published.

Here is an example...

    class ThrusterOut(object):
        alarm_name = 'thruster_out'
        def __init__(self):
            # Keep some knowledge of which thrusters we have working
            self.thruster_layout = wait_for_param('busses')
            self.update_layout = rospy.ServiceProxy('update_thruster_layout', UpdateThrusterLayout)

        def handle(self, time_sent, parameters):
            new_thruster_layout = self.remove_thruster(parameters['thruster_name'])
            rospy.set_param('busses', new_thruster_layout)
            self.update_layout()


Change suggestions are very welcome, this is the first iteration of what will be one day a more mature tool.

# Using System Alarms with C++

To use the Sub 8 alarm system with your C++ package, you'll need to link your executable or library to the sub8_alarm shared library in your package's CMakeLists.txt. To do that, add the following to your CMakeLists.txt: 
* `find_package(sub8_alarm)`
* `DEPENDS sub8_alarm` to `catkin_package(...)`
* `${sub8_alarm_INCLUDE_DIRS}` to `include_directories(...)`
* `${sub8_alarm_LIBRARIES}` to `target_link_libraries(...)`

and to your `package.xml`, add the following: 
* `<build_depend>sub8_alarm</build_depend>`
* `<run_depend>sub8_alarm</run_depend>`

Add the `#include <sub8_alarm/alarm_helpers.h>` include directive to your code to use the `AlarmBroadcaster` and `AlarmRaiser` classes

# Things to worry about

- [Monitor the queue_size in the alarm subscribe](https://github.com/uf-mil/Sub8/pull/23#discussion_r42393150), let's see how many alarms show up at max.


# Plans for 2.0

* No asserts during alarm-raise
* Should we require alarms to include a problem description?
* [ROS diagnostics](http://wiki.ros.org/diagnostics?distro=jade) - could be useful for monitoring hardware systems
* Implement heartbeats
* Each node should expose a generic "System Kill/Unkill" node
* Remove action_required - all alarms should require an action and have a corresponding alarm handler
