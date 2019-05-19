from ros_alarms import HandlerBase, Alarm


class HwKill(HandlerBase):
    alarm_name = 'hw-kill'
    initally_raised = True

    def __init__(self):
        # Alarm server wil set this as the intial state of kill alarm (starts killed)
        self.initial_alarm = Alarm(self.alarm_name, True,
                                   node_name='alarm_server',
                                   problem_description='Initial kill')
