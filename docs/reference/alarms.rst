:mod:`ros_alarms` - Stateful Alarm System
-----------------------------------------

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
