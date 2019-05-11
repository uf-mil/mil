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
    alarm_server = None

    @classmethod
    def _init(cls, alarm_server):
        '''
        Called by the alarm server to give each handler a reference to the alarm server,
        so it can efficiently get and set alarms
        '''
        cls._alarm_server = alarm_server

    @property
    def current_alarm(self):
        '''
        Returns the status of the alarm this handler is registered for
        '''
        return self.get_alarm(self.alarm_name)

    def get_alarm(self, name):
        '''
        Gets the current status of an alarm
        @return The request alarm object, or None if it has not been set
        '''
        return self._alarm_server.alarms.get(name)

    def on_set(self, new_alarm):
        '''
        Called whenever a service request is made to the alarm server to the
        alarm this handler is registered for. Can be used to trigger actions
        before other nodes are notified of the change or to reject the change.
        By default, defers to the raised and cleared functions below.
        @param new_alarm: Alarm message that is requested to change to
        @return Either None, in which case the change is accepted or False,
                in which case the alarm remains the same and the service request fails.
        '''
        currently_raised = self.current_alarm.raised
        if not currently_raised and new_alarm.raised:
            return self.raised(new_alarm)
        elif currently_raised and not new_alarm.raised:
            return self.cleared(new_alarm)
        return

    def raised(self, alarm):
        '''
        Unless on_set is overriden, called whenever a node requests this alarm be raised.
        If it returns False, this request is denied. Otherwise, the alarm is raised
        @param alarm: the new alarm a node had requested to replace the current with
        '''
        return

    def cleared(self, alarm):
        '''
        Unless on_set is overriden, called whenever a node requests this alarm be cleared.
        If it returns False, this request is denied. Otherwise, the alarm is raised
        @param alarm: the new alarm a node had requested to replace the current with
        '''
        return

    def meta_predicate(self, meta_alarm, alarms):
        '''
        Called on an update to one of this alarms's meta alarms, if there are any.

        @param meta_alarm: The alarm object of the meta alarm which was just changed, triggering this callback
        @param alarms: a dictionary with the alarms objects for EVERY meta alarm for this alarm.
                       ex: {'odom-kill' : Alarm('odom-kill', False),
                            'network-loss': Alarm('network-loss', True, 'heartbeat stopped'}

        MUST return either a boolean indiciating if this alarm should now be raised,
             OR an Alarm object which will be the new status of this alarm

        If a boolean is returned, this alarms status will be updated if it differs from the current raised status.

        If an alarm object is returned, this alarm will be updated unconditionaly to the returned object.

        By default, returns True if any meta alarms are raised.
        '''
        return any([alarm.raised for name, alarm in alarms.items()])
