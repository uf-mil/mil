#!/usr/bin/env python
import rospy
from actionlib import SimpleActionClient
from mil_missions.msg import DoMissionAction, DoMissionGoal


class MissionClient(SimpleActionClient):
    '''
    Extends SimpleActionClient to do boostrap of making a mission
    client for you.
    '''
    NS = '/mission'
    LIST_PARAM = '/available_missions'

    def __init__(self):
        SimpleActionClient.__init__(self, self.NS, DoMissionAction)

    @classmethod
    def available_missions(cls):
        '''
        Returns a list of available missions or None if parameter is not set
        '''
        if not rospy.has_param(cls.LIST_PARAM):
            None
        return rospy.get_param(cls.LIST_PARAM)

    def cancel_mission(self):
        '''
        Send a goal to cancel the current mission
        '''
        self.cancel_all_goals()

    def run_mission(self, mission, parameters='', done_cb=None, active_cb=None, feedback_cb=None):
        '''
        Send a goal to start a new mission with the specified parameters
        '''
        goal = DoMissionGoal(mission=mission, parameters=parameters)
        return self.send_goal(goal, done_cb=done_cb, active_cb=active_cb, feedback_cb=feedback_cb)
