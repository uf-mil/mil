#!/usr/bin/env python
from __future__ import division

# from txros import action, util, tf, serviceclient
import txros
# from txros import action, util, tf, serviceclient

from twisted.internet import defer
import txros.tf as tf
import tf.transformations as trns

from mil_misc_tools import text_effects
from sub8_msgs.srv import VisionRequest2DRequest, VisionRequest2D, BMatrix, BMatrixRequest
from std_srvs.srv import SetBool, SetBoolRequest
from geometry_msgs.msg import Quaternion, Point, Vector3, PointStamped, Vector3Stamped, Pose, PoseStamped

import numpy as np

fprint = text_effects.FprintFactory(
    title="START_GATE", msg_color="white").fprint
# @txros.util.cancellableInlineCallbacks
class StartGateMission(object):
    SPEED = 0.3
    HOW_MANY = 20
    def __init__(self, sub):
        self._sub = sub
        self._how_many_points = 0
        self._how_many_normals = 0
        self._center_pt_avg = Point()
        self._normal_avg = Vector3()
        self._found_center_pt = False
        self._found_normal = False
        self._found = False

        self._tf_listener = self._sub._tf_listener

        self._start_away_from_gate = 1;
        self._end_away_from_gate = 0.5;

        self._center_of_gate_sub = self._sub.nh.subscribe('/sub8_start_gate/center', Point, callback = self.process_centers)
        self._normal_of_gate_sub =  self._sub.nh.subscribe('/sub8_start_gate/normal', Vector3, callback = self.process_normals)
        # self._normal_avg = Point( 0.100018282763, 0.221645119556, -0.931298662269)
        # self._center_pt_avg = Vector3(-0.112004441869, 0.0125564267227, 2.06587832023)
        # defer.returnValue(None)

    def process_centers(self, point):
        self._how_many_points = self._how_many_points + 1
        self._center_pt_avg.x += point.x
        self._center_pt_avg.y += point.y
        self._center_pt_avg.z += point.z

        if(self._how_many_points > self.HOW_MANY):
            self._found_center_pt = True

    def process_normals(self, normal):
        self._how_many_normals = self._how_many_normals + 1
        if(normal.z > 0):
            normal.x = -normal.x
            normal.y = -normal.y
            normal.z = -normal.z
        self._normal_avg.x += normal.x
        self._normal_avg.y += normal.y
        self._normal_avg.z += normal.z

        if(self._how_many_normals > self.HOW_MANY):
            self._found_normal = True


    @txros.util.cancellableInlineCallbacks
    def run_mission(self):
        fprint('Starting mission')
        fprint('Assuming camera is facing gate')
        fprint('Leveling off', msg_color="yellow")
        # yield self._sub.move.zero_roll_and_pitch().go()
        while(not self._found_normal and not self._found_center_pt):
            fprint('Waiting for detection...')
            yield self._sub.nh.sleep(1)
        self._found = True
        fprint('Found {} points and {} normals'.format(self._how_many_points, self._how_many_normals))
        self._normal_avg.x/=self._how_many_normals
        self._normal_avg.y/=self._how_many_normals
        self._normal_avg.z/=self._how_many_normals

        self._center_pt_avg.x/=self._how_many_points
        self._center_pt_avg.y/=self._how_many_points
        self._center_pt_avg.z/=self._how_many_points

        fprint('The normal: {} '.format(self._normal_avg))
        fprint('The point: {} '.format(self._center_pt_avg))

        '''
            Go directly in front of gate
        '''
        pose = Pose()
        pose.position = Point(self._center_pt_avg.x + self._normal_avg.x * self._start_away_from_gate, 
                            self._center_pt_avg.y + self._normal_avg.y * self._start_away_from_gate, 
                            self._center_pt_avg.z + self._normal_avg.z * self._start_away_from_gate)
        transform = yield self._tf_listener.get_transform(
            '/map',
            '/front_stereo',
            self._sub.nh.get_time()
        )
        tft = tf.Transform.from_Pose_message(pose)
        transformed_before_gate = transform * tft

        '''
            Go through the gate
        '''
        pose = Pose()
        pose.position = Point(self._center_pt_avg.x + self._normal_avg.x * -self._end_away_from_gate, 
                            self._center_pt_avg.y + self._normal_avg.y * -self._end_away_from_gate,
                            self._center_pt_avg.z + self._normal_avg.z * -self._end_away_from_gate)

        transform = yield self._tf_listener.get_transform(
            '/map',
            '/front_stereo',
            self._sub.nh.get_time()
        )
        tft = tf.Transform.from_Pose_message(pose)
        transformed_after_gate = transform * tft


        fprint('Going infront of gate {}'.format(transformed_before_gate._p))
        yield self._sub.move.set_position(transformed_before_gate._p).go(speed = self.SPEED)

        fprint('Going through gate... YOLO! {}'.format(transformed_after_gate._p))
        yield self._sub.move.set_position(transformed_after_gate._p).go(speed = self.SPEED)

    def search_patten(self):
        yield self._sub.move.yaw_right(0.261799).go()
        yield self._sub.move.pitch_down(0.261799).go()
        yield self._sub.move.yaw_left(0.261799).go()
        yield self._sub.move.pitch_up(0.261799).go()
        yield self._sub.move.yaw_left(0.261799).go()
        yield self._sub.move.pitch_down(0.261799).go()
        yield self._sub.move.yaw_right(0.261799).go()
        yield self._sub.move.pitch_up(0.261799).go()

@txros.util.cancellableInlineCallbacks
def run(sub_singleton):
    start_gate_mission = StartGateMission(sub_singleton)
    # start_gate_mission.search_patten()
    yield start_gate_mission.run_mission()
