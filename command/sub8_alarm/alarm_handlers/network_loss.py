import rospy
from ros_alarms import HandlerBase, AlarmBroadcaster, HeartbeatMonitor
from std_msgs.msg import Header

class NetworkLoss(HandlerBase):
    alarm_name = 'network-loss'

    def __init__(self):
        self.hm = HeartbeatMonitor(self.alarm_name, "/network", node_name="network_loss_kill")
         
    def raised(self, alarm):
        pass

    def cleared(self, alarm):
        pass
