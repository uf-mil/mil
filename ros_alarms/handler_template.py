import rospy

class HandlerBase(object):
    '''
    Listens for an alarm with this `alarm_name`.
    When that alarm is raised, self.handle will be called.
    When that alarm is cleared, self.cancel will be called.

    All alarm handlers must inherit from this base class in order to be registered.
    '''
    alarm_name = 'generic_name'
    severity_required = -1

    def raised(self, alarm):
        rospy.logwarn("No raised function defined for '{}'.".format(alarm.alarm_name))
        return

    def cleared(self, alarm):
        rospy.logwarn("No cleared function defined for '{}'.".format(alarm.alarm_name))
        return

