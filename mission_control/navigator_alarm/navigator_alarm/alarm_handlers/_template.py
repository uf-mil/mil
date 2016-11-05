import rospy
from navigator_alarm import AlarmBroadcaster

class HandlerBase(object):
    '''
    We will listen for an alarm with this `alarm_name`.
    When that alarm is raised, self.handle will be called.
    When that alarm is cleared, self.cancel will be called.
    '''
    alarm_name = 'generic_name'
    alarm_broadcaster = AlarmBroadcaster()

    def handle(self, alarm, time_sent, parameters):
        rospy.logwarn("No handle function defined for '{}'.".format(alarm.alarm_name))
        return

    def cancel(self, alarm, time_sent, parameters):
        rospy.logwarn("No cancel function defined for '{}'.".format(alarm.alarm_name))
        return
