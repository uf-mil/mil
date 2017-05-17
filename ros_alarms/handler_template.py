import rospy


class HandlerBase(object):

    '''
    Listens for an alarm with this `alarm_name`.
    When that alarm is raised, self.raised will be called.
    When that alarm is cleared, self.cleared will be called.
    See the comments below for self.meta_predicate.

    All alarm handlers must inherit from this base class in order to be registered.
    '''
    alarm_name = 'generic-name'
    severity_required = (0, 5)
    initally_raised = False

    def raised(self, alarm):
        rospy.logwarn("No raised function defined for '{}'.".format(alarm.alarm_name))
        return

    def cleared(self, alarm):
        rospy.logwarn("No cleared function defined for '{}'.".format(alarm.alarm_name))
        return

    def meta_predicate(self, meta_alarm, sub_alarms):
        '''ONLY APPLICABLE FOR META ALARMS
        Returns `True` or `False` to raise or clear (respectively) this meta alarm given
            a dictionary of all of it's sub alarms:
                {sub_alarm_name1: alarm_msg, sub_alarm_name2: alarm_msg, ...}
        '''
        return any(sub_alarms.values())
