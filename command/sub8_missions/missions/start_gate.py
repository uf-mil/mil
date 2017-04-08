#!/usr/bin/env python
from __future__ import division

# from txros import action, util, tf, serviceclient
import txros
import rospkg
import time
from tf import transformations

from uf_common.msg import MoveToAction, PoseTwistStamped, Float64Stamped
from sub8 import pose_editor
import sub8_tools
from sub8_msgs.srv import VisionRequest, VisionRequestRequest, VisionRequest2DRequest, VisionRequest2D, BMatrix, BMatrixRequest
from std_srvs.srv import SetBool, SetBoolRequest
from nav_msgs.msg import Odometry

import numpy as np
from twisted.internet import defer
import os
import yaml

from sub8 import sub_singleton


fprint = sub8_tools.text_effects.FprintFactory(
title="START_GATE", msg_color="white").fprint

class StartGateMission(object):

    FOUND_START_GATE = False
    SPEED = 0.3

    def __init__(self, sub):
        self.sub_singleton = sub

    @txros.util.cancellableInlineCallbacks
    def run_mission(self):
        fprint('Starting mission')
        fprint('Assuming camera is facing gate')
        fprint('Leveling off', msg_color="yellow")
        yield self.sub_singleton.move.zero_roll_and_pitch().go()
        fprint('Going down', msg_color="yellow")
        yield self.sub_singleton.move.down(.7).go()

        start_gate_enable = yield self.sub_singleton.nh.get_service_client('/vision/start_gate/enable', SetBool)
        fprint('Turning on vision service')
        yield start_gate_enable(SetBoolRequest(data=True))

        start_gate_search = yield self.sub_singleton.nh.get_service_client('/vision/start_gate/pose', VisionRequest2D)
        fprint("Searching for start gate pose")
        start_gate_search_res = yield start_gate_search(VisionRequest2DRequest(target_name=''))

        if not start_gate_search_res.found:
            fprint("Waiting a few seconds and trying again:")
            time.sleep(3)
            start_gate_search_res = yield start_gate_search(VisionRequest2DRequest(target_name=''))
        # This is to reset the buffer
        yield start_gate_enable(SetBoolRequest(data=False))
        yield start_gate_enable(SetBoolRequest(data=True))
        if not start_gate_search_res.found:
            fprint("Running search pattern")
            while not self.FOUND_START_GATE and start_gate_search_res.pose.x == 0:
                fprint(self.FOUND_START_GATE)
                time.sleep(1)
                fprint("AGAIN - Searching for start gate pose")
                self.start_gate_find()
                yield self.search_pattern()
        
        fprint("Found start gate: " + str(start_gate_search_res.pose))
        
        start_gate_distance_request = yield self.sub_singleton.nh.get_service_client('/vision/start_gate/distance', BMatrix)
        start_gate_distance = yield start_gate_distance_request(BMatrixRequest())
        fprint("Distance: " + str(start_gate_distance.B[0]))


        yield self.align_for_dummies( start_gate_search_res)
        fprint("YOLO -- MOVING FORWARD")

        # while(self.FOUND_START_GATE):
            # self.start_gate_find()
            # yield self.sub_singleton.move.forward(.3).zero_roll_and_pitch().go(speed=self.SPEED)
        yield self.sub_singleton.move.forward(start_gate_distance.B[0] + 0.5).zero_roll_and_pitch().go(speed=self.SPEED)

        fprint("Finished")

    @txros.util.cancellableInlineCallbacks
    def start_gate_find(self):
        start_gate_search = yield self.sub_singleton.nh.get_service_client('/vision/start_gate/pose', VisionRequest2D)
        start_gate_search_res = yield start_gate_search(VisionRequest2DRequest(target_name=''))
        self.FOUND_START_GATE = start_gate_search_res.found

    @txros.util.cancellableInlineCallbacks
    def search_pattern(self):
        if not self.FOUND_START_GATE:
            fprint("Moving up .1", msg_color="yellow")
            yield self.sub_singleton.move.up(.1).zero_roll_and_pitch().go(speed=self.SPEED)
            fprint("Moving down .3", msg_color="yellow")
            yield self.sub_singleton.move.down(.3).zero_roll_and_pitch().go(speed=self.SPEED)
        if not self.FOUND_START_GATE:
            fprint("Moving left .3", msg_color="yellow")
            yield self.sub_singleton.move.left(.3).zero_roll_and_pitch().go(speed=self.SPEED)
            fprint("Moving right .3", msg_color="yellow")
            yield self.sub_singleton.move.right(.3).zero_roll_and_pitch().go(speed=self.SPEED)


    @txros.util.cancellableInlineCallbacks
    def align_for_dummies(self, start_gate_search_res):
        error_tolerance = 0.1
        deviation = ((start_gate_search_res.pose.x - start_gate_search_res.camera_info.width/2)/start_gate_search_res.camera_info.width,(start_gate_search_res.pose.y - start_gate_search_res.camera_info.height/2)/start_gate_search_res.camera_info.height)
           
        fprint("Alignment -- Deviation: " + str(deviation), msg_color="cyan")
        while(np.abs(deviation[0]) > error_tolerance):
            fprint("Moving right " + str(.1*np.sign(deviation[0])), msg_color="yellow")
            yield self.sub_singleton.move.right(.1*np.sign(deviation[0])).zero_roll_and_pitch().go(speed=self.SPEED)
        while(np.abs(deviation[1]) > error_tolerance):
            fprint("Moving up " + str(.1*np.sign(deviation[1])), msg_color="yellow")
            yield self.sub_singleton.move.up(.1*np.sign(deviation[1])).zero_roll_and_pitch().go(speed=self.SPEED)
        yield fprint("Finished Alignment -- Deviation: " + str(deviation), msg_color="cyan")




@txros.util.cancellableInlineCallbacks
def run(sub_singleton):
    start_gate_mission = StartGateMission(sub_singleton)
    yield start_gate_mission.run_mission()