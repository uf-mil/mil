#!/usr/bin/env python
import rospy
from ros_alarms import HandlerBase, Alarm
from actionlib import SimpleActionClient, TerminalState
from mil_msgs.msg import BagOnlineAction, BagOnlineGoal
import os
from mil_tasks_core import TaskClient


class Kill(HandlerBase):
    alarm_name = 'kill'

    def __init__(self):
        self._killed = False
        self.initial_alarm = Alarm(self.alarm_name, True,
                                   node_name='alarm_server',
                                   problem_description='Initial kill')
        self.bag_client = SimpleActionClient("/online_bagger/bag", BagOnlineAction)
        self.task_client = TaskClient()
        self.first = True

    def _online_bagger_cb(self, status, result):
        if status == 3:
            rospy.loginfo('KILL BAG WRITTEN TO {}'.format(result.filename))
        else:
            rospy.logwarn('KILL BAG {}, status: {}'.format(TerminalState.to_string(status), result.status))

    def _kill_task_cb(self, status, result):
        if status == 3:
            rospy.loginfo('Killed task success!')
            return
        rospy.logwarn('Killed task failed ({}): {}'.format(TerminalState.to_string(status), result.result))

    def raised(self, alarm):
        self._killed = True
        if self.first:
            self.first = False
            return
        if 'BAG_ALWAYS' not in os.environ or 'bag_kill' not in os.environ:
            rospy.logwarn('BAG_ALWAYS or BAG_KILL not set. Not making kill bag.')
        else:
            goal = BagOnlineGoal(bag_name='kill.bag')
            goal.topics = os.environ['BAG_ALWAYS'] + ' ' + os.environ['bag_kill']
            self.bag_client.send_goal(goal, done_cb=self._online_bagger_cb)
        self.task_client.run_task('Killed', done_cb=self._kill_task_cb)

    def cleared(self, alarm):
        self._killed = False

    def meta_predicate(self, meta_alarm, alarms):
        ignore = []

        # Don't kill on low battery, only on critical
        # if alarms['battery-voltage'].raised and alarms['battery-voltage'].severity < 2:
        #     ignore.append('battery-voltage')

        # Raised if any alarms besides the two above are raised
        raised = [name for name, alarm in alarms.items() if name not in ignore and alarm.raised]
        if len(raised):
            return Alarm('kill', True, node_name=rospy.get_name(),
                         problem_description='Killed by meta alarm(s) ' + ', '.join(raised),
                         parameters={'Raised': raised})
        return self._killed
