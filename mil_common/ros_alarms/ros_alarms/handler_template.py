from typing import Optional, Union

from ros_alarms_msgs.msg import Alarm as AlarmMsg

from .alarms import Alarm


class HandlerBase:
    """
    Listens for an alarm with this `alarm_name`. When that alarm is raised,
    self.raised will be called. When that alarm is cleared, self.cleared will
    be called. See the comments below for self.meta_predicate.

    All alarm handlers must inherit from this base class in order to be registered.
    """

    alarm_name = "generic-name"
    severity_required = (0, 5)
    alarm_server = None

    @classmethod
    def _init(cls, alarm_server):
        """
        Called by the alarm server to give each handler a reference to the alarm server,
        so it can efficiently get and set alarms.

        Args:
            alarm_server (AlarmServer): An existing AlarmServer instance.
        """
        cls._alarm_server = alarm_server

    @property
    def current_alarm(self) -> Optional[Alarm]:
        """
        Returns the status of the alarm this handler is registered for.

        Returns:
            Optional[Alarm]: The current alarm instance, if it exists, otherwise ``None``.
        """
        return self.get_alarm(self.alarm_name)

    def get_alarm(self, name: str) -> Optional[Alarm]:
        """
        Gets the current status of an alarm.

        Returns:
            Optional[Alarm]: The request alarm object, or None if it has not been set.
        """
        return self._alarm_server.alarms.get(name)

    def on_set(self, new_alarm: AlarmMsg) -> Optional[bool]:
        """
        Called whenever a service request is made to the alarm server to the
        alarm this handler is registered for. Can be used to trigger actions
        before other nodes are notified of the change or to reject the change.
        By default, defers to the raised and cleared functions below.

        Args:
            new_alarm (ros_alarms.msg._Alarm.Alarm): Alarm message that is
                requested to have this alarm change to.

        Returns:
            Optional[bool]: Either ``None``, in which case the change is accepted or ``False``,
            in which case the alarm remains the same and the service request fails.
        """
        if new_alarm.raised:
            return self.raised(new_alarm)
        else:
            return self.cleared(new_alarm)

    def raised(self, alarm: AlarmMsg):
        """
        Unless on_set is overridden, called whenever a node requests this alarm be raised.
        If it returns False, this request is denied. Otherwise, the alarm is raised.

        Args:
            alarm (ros_alarms.msg._Alarm.Alarm): The new alarm a node had requested to replace the
                current with.
        """
        return

    def cleared(self, alarm: AlarmMsg):
        """
        Unless on_set is overridden, called whenever a node requests this alarm be cleared.
        If it returns False, this request is denied. Otherwise, the alarm is raised

        Args:
            alarm (ros_alarms.msg._Alarm.Alarm): The new alarm a node had requested to replace the
                current with.
        """
        return

    def meta_predicate(self, meta_alarm: Alarm, alarms) -> Union[bool, Alarm]:
        """
        Called on an update to one of this alarms's meta alarms, if there are any.
        By default, returns True if any meta alarms are raised.

        Args:
            meta_alarm (ros_alarms.Alarm): The alarm object of the meta alarm.
            alarms (Dict[str, ros_alarms.Alarm]): A dictionary mapping the name
                of each child alarm to its Alarm object.

        Returns:
            Union[bool, ros_alarms.Alarm]: Returns a bool indicating if the alarm
            should be raised, or a new Alarm object which the calling alarm
            should update itself to.
        """
        return any(alarm.raised for name, alarm in alarms.items())
