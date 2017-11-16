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

    def raised(self, alarm):
        rospy.logwarn("No raised function defined for '{}'.".format(alarm.alarm_name))
        return

    def cleared(self, alarm):
        rospy.logwarn("No cleared function defined for '{}'.".format(alarm.alarm_name))
        return

    def meta_predicate(self, meta_alarm, sub_alarms):
        '''
        Called on an update to one of this alarms's meta alarms, if there are any.

        @param meta_alarm: The alarm object of the meta alarm which was just changed, triggering this callback
        @param alarms: a dictionary with the alarms objects for EVERY meta alarm for this alarm.
                       ex: {'odom-kill' : Alarm('odom-kill', False), 'network-loss': Alarm('network-loss', True, 'heartbeat stopped'}

        MUST return either a boolean indiciating if this alarm should now be raised,
             OR an Alarm object which will be the new status of this alarm

        If a boolean is returned, this alarms status will be updated if it differs from the current raised status.

        If an alarm object is returned, this alarm will be updated unconditionaly to the returned object.

        By default, returns True if any meta alarms are raised.
        '''
        return any([alarm.raised for name, alarm in alarms.items()])
