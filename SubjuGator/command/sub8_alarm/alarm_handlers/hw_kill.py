from ros_alarms import HandlerBase, Alarm


class HwKill(HandlerBase):
    """
    Alarm (inheriting from :class:`ros_alarms.HandlerBase`) to represent that the sub experienced
    a hardware kill.

    Attributes:
        alarm_name (str): The name of the alarm. Equal to ``hw-kill``.
    """
    alarm_name = 'hw-kill'

    def __init__(self):
        pass

