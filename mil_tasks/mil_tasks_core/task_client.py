#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from mil_tasks.msg import DoTaskAction, DoTaskGoal


class TaskClient(SimpleActionClient):
    '''
    Extends SimpleActionClient to do boostrap of making a task
    client for you.
    '''
    NS = '/task'
    LIST_PARAM = '/available_tasks'

    def __init__(self):
        SimpleActionClient.__init__(self, self.NS, DoTaskAction)

    @classmethod
    def available_tasks(cls):
        '''
        Returns a list of available tasks or None if parameter is not set
        '''
        if not rospy.has_param(cls.LIST_PARAM):
            None
        return rospy.get_param(cls.LIST_PARAM)

    def cancel_task(self):
        '''
        Send a goal to cancel the current task
        '''
        self.cancel_all_goals()

    def run_task(self, task, parameters='', done_cb=None, active_cb=None, feedback_cb=None):
        '''
        Send a goal to start a new task with the specified parameters
        '''
        goal = DoTaskGoal(task=task, parameters=parameters)
        return self.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
