:mod:`ros_alarms` - Stateful Alarm System
-----------------------------------------
**Overview**

In the realm of building dependable control systems, the imperative for error detection and effective error-handling mechanisms cannot be overstated. Within this context, MIL (Mission Impossible Labs) presents a robust solution in the form of a dynamic live alarm system. This alarm system operates discreetly in the background of both the robot's mission and driver codebases, ready to be activated upon the emergence of errors. Notably, the alarm code doesn't solely serve to identify and address errors; it can also adeptly manage changes that extend beyond error scenarios.

**ROS Alarms: A Service-Oriented Architecture**

The architecture of ROS alarms distinguishes itself by employing a service-oriented model rather than the customary topic-based approach. In ROS, Services act as the conduits for interaction between nodes, functioning in a request-response manner. While ROS topics enable asynchronous data exchange, services facilitate nodes in seeking specific actions or information from other nodes, awaiting a subsequent response before proceeding. This proves especially valuable in tasks necessitating direct engagement, such as data retrieval or computations.

**Components of a ROS Service**

A ROS service structure is bifurcated into two pivotal components:

1. **Server**: This node takes on the role of service provision, awaiting incoming requests from other nodes. Essentially, it executes the operation stipulated by the service.

2. **Client**: In contrast, this node is responsible for transmitting requests to the service server, then patiently awaiting the corresponding response.

For a concrete illustration, consider the case of a service designed to sum two integers:

Server Implementation:
```python
#!/usr/bin/env python

import rospy
from service_example.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req):
    sum_result = req.a + req.b
    rospy.loginfo(f"Adding {req.a} + {req.b} = {sum_result}")
    return AddTwoIntsResponse(sum_result)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    service = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    rospy.loginfo("Ready to add two ints.")
    rospy.spin()

if __name__ == '__main__':
    add_two_ints_server()

```

Client Implementation:
```python
#!/usr/bin/env python

import rospy
from service_example.srv import AddTwoInts, AddTwoIntsRequest

def add_two_ints_client(a, b):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        response = add_two_ints(a, b)
        return response.sum
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    rospy.init_node('add_two_ints_client')
    a = 10
    b = 5
    result = add_two_ints_client(a, b)
    rospy.loginfo(f"Sum of {a} and {b} is {result}")

```

**Alarm System Logic**

The alarm system's functionality is more intricate than the preceding example. In this scenario, the server is engineered to manage not numeric calculations, but the tasks of updating, setting, and querying alarm data. Unlike the single-client model, ROS alarms encompass two distinct types of clients: the alarm broadcaster and the alarm listener. The broadcaster initializes and triggers alarms in response to errors or changes, while the listener monitors the broadcaster's activity and activates designated callback functions when alarms are raised.

To peruse the detailed alarm system code, refer to the repository: [https://github.com/uf-mil/mil/tree/master/mil_common/ros_alarms](https://github.com/uf-mil/mil/tree/master/mil_common/ros_alarms)

To successfully leverage alarms, the initiation of both the broadcaster and listener is requisite. The listener should be configured to execute a predefined callback function, addressing errors or changes detected by the broadcaster. Within your codebase, error detection and alarm raising procedures should be integrated. If orchestrated adeptly, the callback function will be automatically invoked, underscoring successful error mitigation.

For a practical example of this workflow, visit: [https://github.com/uf-mil/mil/blob/master/mil_common/ros_alarms/test/rospy/callback_test.py](https://github.com/uf-mil/mil/blob/master/mil_common/ros_alarms/test/rospy/callback_test.py)

**Applications and Context**

The applications of ROS alarms span various contexts, with one notable application residing in the control of the submersible vehicle's thrust and kill board. The thrust and kill board, responsible for the sub's electronic operations, is integrally associated with ROS alarms. Upon the board's activation or deactivation (hard or soft kill), alarms are invoked to apprise users of these changes. The listener's callback function comes into play, ensuring that alarms are updated in alignment with the board's current state. This intricate process, triggered each time the board is deactivated, creates a system whereby users are continually informed about the board's status changes – essentially manifesting a dynamic live alarm system.

To delve into the implementation, visit: [https://github.com/uf-mil/mil/blob/master/SubjuGator/drivers/sub8_thrust_and_kill_board/sub8_thrust_and_kill_board/handle.py](https://github.com/uf-mil/mil/blob/master/SubjuGator/drivers/sub8_thrust_and_kill_board/sub8_thrust_and_kill_board/handle.py)
.. automodule:: ros_alarms

Python
^^^^^^

Various Python classes have been built around making the alarm system easier to
interface with between nodes.

For example, these classes can be used to immediately control alarms without
the need to interface with topics or services:

.. code-block:: python

    >>> # Assume that the alarm_server node has been started
    >>> from ros_alarms import Alarm, AlarmBroadcaster, AlarmListener
    >>> broadcaster = AlarmBroadcaster("test-alarm")
    >>> listener = AlarmListener("test-alarm")
    >>> def callback(alarm: Alarm):
    ...     print(f"An alarm with the name {alarm.alarm_name} was called.")
    >>> listener.add_callback(callback, call_when_raised = True)
    >>> broadcaster.raise_alarm()
    An alarm with the name test-alarm was called.

Alarm
~~~~~
.. attributetable:: ros_alarms.Alarm

.. autoclass:: ros_alarms.Alarm
    :members:

AlarmServer
~~~~~~~~~~~
.. attributetable:: ros_alarms.nodes.alarm_server.AlarmServer

.. autoclass:: ros_alarms.nodes.alarm_server.AlarmServer
    :members:

AlarmBroadcaster
~~~~~~~~~~~~~~~~
.. attributetable:: ros_alarms.AlarmBroadcaster

.. autoclass:: ros_alarms.AlarmBroadcaster
    :members:

AlarmListener
~~~~~~~~~~~~~
.. attributetable:: ros_alarms.AlarmListener

.. autoclass:: ros_alarms.AlarmListener
    :members:

HeartbeatMonitor
~~~~~~~~~~~~~~~~
.. attributetable:: ros_alarms.HeartbeatMonitor

.. autoclass:: ros_alarms.HeartbeatMonitor
    :members:

HandlerBase
~~~~~~~~~~~
.. attributetable:: ros_alarms.HandlerBase

.. autoclass:: ros_alarms.HandlerBase
    :members:


C++
^^^

AlarmProxy
~~~~~~~~~~
.. cppattributetable:: ros_alarms::AlarmProxy

.. doxygenstruct:: ros_alarms::AlarmProxy

AlarmBroadcaster
~~~~~~~~~~~~~~~~
.. cppattributetable:: ros_alarms::AlarmBroadcaster

.. doxygenclass:: ros_alarms::AlarmBroadcaster

AlarmListener
~~~~~~~~~~~~~
.. cppattributetable:: ros_alarms::AlarmListener

.. doxygenclass:: ros_alarms::AlarmListener

ListenerCb
~~~~~~~~~~
.. cppattributetable:: ros_alarms::ListenerCb

.. doxygenstruct:: ros_alarms::ListenerCb

.. Causes sphinx-doc/sphinx#10152
.. HeartbeatMonitor
.. ~~~~~~~~~~~~~~~~
.. .. cppattributetable:: ros_alarms::HeartbeatMonitor

.. .. doxygenclass:: ros_alarms::HeartbeatMonitor

Subjugator-specific
^^^^^^^^^^^^^^^^^^^

BusVoltage
~~~~~~~~~~
.. attributetable:: alarm_handlers.BusVoltage

.. autoclass:: alarm_handlers.BusVoltage
    :members:

HeightOverBottom
~~~~~~~~~~~~~~~~
.. attributetable:: alarm_handlers.HeightOverBottom

.. autoclass:: alarm_handlers.HeightOverBottom
    :members:

HwKill
~~~~~~
.. attributetable:: alarm_handlers.HwKill

.. autoclass:: alarm_handlers.HwKill
    :members:

Kill
~~~~
.. attributetable:: alarm_handlers.Kill

.. autoclass:: alarm_handlers.Kill
    :members:

NetworkLoss
~~~~~~~~~~~
.. attributetable:: alarm_handlers.NetworkLoss

.. autoclass:: alarm_handlers.NetworkLoss
    :members:

OdomKill
~~~~~~~~
.. attributetable:: alarm_handlers.OdomKill

.. autoclass:: alarm_handlers.OdomKill
    :members:

ThrusterOut
~~~~~~~~~~~~
.. attributetable:: alarm_handlers.ThrusterOut

.. autoclass:: alarm_handlers.ThrusterOut
    :members:
