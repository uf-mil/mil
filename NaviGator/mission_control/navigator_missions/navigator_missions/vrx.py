#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from vrx_gazebo.msg import Task
from robot_localization.srv import FromLL, FromLLRequest
from geographic_msgs.msg import GeoPoseStamped, GeoPath
from navigator import Navigator
from mil_tools import rosmsg_to_numpy

___author___ = "Kevin Allen"


class Vrx(Navigator):
    def __init__(self):
        super(Vrx, self).__init__()

    @classmethod
    def init(cls):
        cls.from_lla = cls.nh.get_service_client('/fromLL', FromLL)
        cls.station_keep_goal = cls.nh.subscribe('/vrx/station_keeping/goal', GeoPoseStamped)
        cls.task_info_sub = cls.nh.subscribe("/vrx/task/info", Task)
        cls.wayfinding_path_sub = cls.nh.subscribe("/vrx/wayfinding/waypoints", GeoPath)

    #@txros.util.cancellableInlineCallbacks
    def cleanup(self):
        pass

    @txros.util.cancellableInlineCallbacks
    def geo_pose_to_enu_pose(self, geo):
        self.send_feedback('Waiting for LLA conversion')
        enu_msg = yield self.from_lla(FromLLRequest(ll_point=geo.position))
        position_enu = rosmsg_to_numpy(enu_msg.map_point)
        orientation_enu = rosmsg_to_numpy(geo.orientation)
        defer.returnValue((position_enu, orientation_enu))

    @txros.util.cancellableInlineCallbacks
    def get_latching_msg(self, sub):
        msg = yield sub.get_last_message()
        if msg is None:
            msg = yield sub.get_next_message()
        defer.returnValue(msg)

    @txros.util.cancellableInlineCallbacks
    def run_station_keeping(self):
        yield self.wait_for_task_such_that(lambda task: task.state in ['ready', 'running'])
        self.send_feedback('Waiting for station keeping goal')
        goal_msg = yield self.get_latching_msg(self.station_keep_goal)
        goal_pose = yield self.geo_pose_to_enu_pose(goal_msg.pose)
        self.send_feedback('Going to {}'.format(goal_pose))
        yield self.move.set_position(goal_pose[0]).set_orientation(goal_pose[1]).go()

    @txros.util.cancellableInlineCallbacks
    def wait_for_task_such_that(self, f):
        while True:
            msg = yield self.task_info_sub.get_next_message()
            if f(msg):
                defer.returnValue(None)

    @txros.util.cancellableInlineCallbacks
    def run_wayfinding(self):
        self.send_feedback('Waiting for task to start')
        yield self.wait_for_task_such_that(lambda task: task.state in ['ready', 'running'])
        path_msg = yield self.get_latching_msg(self.wayfinding_path_sub)
        poses = [ (yield self.geo_pose_to_enu_pose(geo_pose.pose)) for geo_pose in path_msg.poses]
        position = self.pose[0]
        poses = sorted(poses, key=lambda pose: np.linalg.norm(pose[0] - position))
        self.send_feedback('Sorted poses' + str(poses))
        yield self.wait_for_task_such_that(lambda task: task.state in ['running'])
        for pose in poses:
            self.send_feedback('Gong to {}'.format(pose))
            yield self.move.set_position(pose[0]).set_orientation(pose[1]).go()


    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        msg = yield self.task_info_sub.get_next_message()
        task_name = msg.name
        if task_name == 'stationkeeping':
            yield self.run_station_keeping()
        elif task_name == 'wayfinding':
            yield self.run_wayfinding()
        defer.returnValue(msg)
