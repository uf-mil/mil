"""
Stateful alarm system that allows for the immediate kill of a robotic machine.

This package implements a live alarm system, which requires that there is an
alarm server consistently alive to field requests. This alarm server maintains
the status of all of the robot's alarms and handles requested get and set
operations.

The :class:`ros_alarms.Alarm` class serves as the data structure used by the alarm
server to store data about individual alarms during its lifecycle. This class
should not be constructed by clients or other packages. However, this class
may be passed into callbacks registered with broadcasters or listeners.

Rather, other clients should interface with this library through the
:class:`ros_alarms.AlarmListener` or :class:`ros_alarms.AlarmBroadcaster` classes,
or the services provided by the alarm server. These classes wrap around the topics
published by the running alarm server to provide more useful Pythonic features.

.. container:: services

    .. describe:: /alarm/get

        Handles a string argument, the name of the alarm that is requested.
        Returns a stamped message containing the alarm information.

    .. describe:: /alarm/set

        Handles an alarm argument that is used to overwrite the previous alarm
        information. This argument should contain all information relevant to
        the alarm.

        Because you need to pass an entire alarm object, it is not recommended
        to use this service in other places. Instead, try using the
        :class:`~ros_alarms.AlarmBroadcaster` class.

.. container:: topics

    .. describe:: /alarm/updates

        Topic publishing any getting/setting updates that occur on alarms.
"""
from .alarms import (
    Alarm,
    AlarmBroadcaster,
    AlarmListener,
    HeartbeatMonitor,
    parse_json_str,
)
from .handler_template import HandlerBase
from .tx_alarms import TxAlarmBroadcaster, TxAlarmListener, TxHeartbeatMonitor
