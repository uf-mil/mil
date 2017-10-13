#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from mil_tasks.msg import DoTaskAction, DoTaskGoal


class TaskClient(SimpleActionClient):
    NS = '/task'
    LIST_PARAM = '/available_tasks'
    def __init__(self):
        SimpleActionClient.__init__(self, self.NS, DoTaskAction)

    @classmethod
    def available_tasks(cls):
        if not rospy.has_param(cls.LIST_PARAM):
            None
        return rospy.get_param(cls.LIST_PARAM)

    def cancel_task(self):
        self.cancel_all_goals()

    def run_task(self, task, parameters='', done_cb=None, active_cb=None, feedback_cb=None):
        goal = DoTaskGoal(task=task, parameters=parameters)
        return self.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
