import rospy
from ros_alarms import HandlerBase, AlarmBroadcaster, HeartbeatMonitor
from std_srvs.srv import Trigger

class NetworkLoss(HandlerBase):
    alarm_name = 'network-loss'

    def __init__(self):
        self.kill = AlarmBroadcaster("network-kill", node_name="network_loss_kill")
        self.go_auto = rospy.ServiceProxy("/go_auto", Trigger)
        self.hm = HeartbeatMonitor(self.alarm_name, "/network", node_name="network_loss_kill")
         
    def raised(self, alarm):
        if rospy.get_param("autonomous", False):
            # What to do when cable is unplugged and autonomous is True 
            self.go_auto()
        else:
            # What to do when cable is unplugged and autonomous is False 
            self.kill.raise_alarm(severity=5)
        
    def cleared(self, alarm):
        self.kill.clear_alarm()
