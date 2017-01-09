import rospy
from ros_alarms import AlarmBroadcaster, HandlerBase
from uf_common.msg import Float64Stamped

class HeightOverBottom(HandlerBase):
    alarm_name = "height_over_bottom"
    
    def __init__(self):
        self._killed = False
        self._update_height()

        self.ab = AlarmBroadcaster(self.alarm_name, node_name="height_over_bottom_kill")
        
        # Keep track of the current height
        self._last_height = 100 
        set_last_height = lambda msg: setattr(self, "_last_height", msg.data)
        rospy.Subscriber("/dvl/range", Float64Stamped, set_last_height)
       
        # Every 5 seconds, check for an updated height param. A pseudo dynamic reconfig thing.
        rospy.Timer(rospy.Duration(5), self._update_height)
        
        # This should smooth out random dips below the limit 
        rospy.Timer(rospy.Duration(0.5), self._do_check)

    def _do_check(self, *args):
        if self._last_height <= self._height_to_kill and not self._killed:
            rospy.logwarn("SUB TOO LOW!") 
            self.ab.raise_alarm(problem_description="The sub was too low: {}".format(self._last_height),
                                parameters={"height": self._last_height},
                                severity=0
            )
        elif self._last_height >= self._height_to_kill and self._killed:
            rospy.logwarn("REVIVING")
            self.ab.clear_alarm()
    
    def _update_height(self, *args):
        self._height_to_kill = rospy.get_param("/height_over_bottom", 0.4)

    def raised(self, alarm):
        self._killed = True

    def cleared(self, alarm):
        self._killed = False
