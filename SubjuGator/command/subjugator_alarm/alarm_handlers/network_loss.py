from ros_alarms import HandlerBase, HeartbeatMonitor
from std_msgs.msg import Header


class NetworkLoss(HandlerBase):
    """
    Alarm class used to represent network loss.

    Attributes:
        alarm_name (str): The name of the alarm. Set to ``network-loss``.
        hm (ros_alarms.HeartbeatMonitor): The heartbeat monitor to monitor the
          network.
    """

    alarm_name = "network-loss"

    def __init__(self):
        self.hm = HeartbeatMonitor(
            self.alarm_name,
            "/network",
            Header,
            prd=0.8,
            node_name="network_loss_kill",
        )

    def raised(self, alarm) -> None:
        """
        Called when the alarm is raised. Does nothing.
        """

    def cleared(self, alarm) -> None:
        """
        Called when the alarm is cleared. Does nothing.
        """
