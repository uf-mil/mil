import rospy
from ros_alarms import HandlerBase
from kill_handling.broadcaster import KillBroadcaster

class Kill(HandlerBase):
    alarm_name = 'kill'
    initally_raised = True

    def __init__(self):
        # Keep some knowledge of which thrusters we have working
        self.kb = KillBroadcaster(id='alarm-kill', description='Kill by alarm')
        self.alarms = {}
        self._killed = False

    def raised(self, alarm):
        self._killed = True
        self.alarms[alarm.alarm_name] = True
        self.kb.send(active=True)

    def cleared(self, alarm):
        self._killed = False
        self.alarms[alarm.alarm_name] = False

        # Make sure that ALL alarms that caused a kill have been cleared
        if not any(self.alarms.values()):
            self.kb.send(active=False)
    
    def meta_predicate(self, meta_alarm, sub_alarms):
        # Stay killed until user clears
        if self._killed:
            return True

        # Battery too low
        if sub_alarms["bus-voltage"].severity == 5:
            return True
        
        # If we lose network but don't want to go autonomous
        if sub_alarms["network-loss"].raised and not rospy.get_param("autonomous", False):
            return True
        
        # Raised if any alarms besides the two above are raised
        return any([alarm.raised for name, alarm in sub_alarms.items() \
                    if name not in ["bus-voltage", "network-loss"]])


