from ros_alarms import HandlerBase, HeartbeatMonitor
from std_msgs.msg import Header


class NetworkLoss(HandlerBase):
    alarm_name = 'network-loss'

    def __init__(self):
        self.hm = HeartbeatMonitor(self.alarm_name, '/network', Header, node_name='alarm_server', prd=1.0)

    def raised(self, alarm):
        pass

    def cleared(self, alarm):
        pass
