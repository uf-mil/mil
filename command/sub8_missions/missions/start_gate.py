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
from sub8_msgs.srv import VisionRequest, VisionRequestRequest, VisionRequest2DRequest, VisionRequest2D
from std_srvs.srv import SetBool, SetBoolRequest
from nav_msgs.msg import Odometry

import numpy as np
from twisted.internet import defer
import os
import yaml

from sub8 import sub_singleton

FOUND_START_GATE = False
SPEED = 0.3

fprint = sub8_tools.text_effects.FprintFactory(
    title="START_GATE", msg_color="white").fprint

@txros.util.cancellableInlineCallbacks
def run(sub_singleton):
    fprint('Starting mission')
    fprint('Assuming camera is facing gate')
    fprint('Leveling off', msg_color="yellow")
    yield sub_singleton.move.zero_roll_and_pitch().go()

    start_gate_enable = yield sub_singleton.nh.get_service_client('/vision/start_gate/enable', SetBool)
    fprint('Turning on vision service')
    yield start_gate_enable(SetBoolRequest(data=True))

    start_gate_search = yield sub_singleton.nh.get_service_client('/vision/start_gate/pose', VisionRequest2D)
    fprint("Searching for start gate pose")
    start_gate_search_res = yield start_gate_search(VisionRequest2DRequest(target_name=''))

    if not start_gate_search_res.found:
        fprint("Waiting a few seconds and trying again:")
        time.sleep(3)
        start_gate_search_res = yield start_gate_search(VisionRequest2DRequest(target_name=''))
    # yield start_gate_enable(SetBoolRequest(data=False))
    # yield start_gate_enable(SetBoolRequest(data=True))
    if not start_gate_search_res.found:
        fprint("Running search pattern")
        global FOUND_START_GATE
        while not FOUND_START_GATE and start_gate_search_res.pose.x == 0:
            fprint(FOUND_START_GATE)
            time.sleep(1)
            fprint("AGAIN - Searching for start gate pose")
            start_gate_find(sub_singleton)
            yield search_pattern(sub_singleton)
    
    fprint("Found start gate: " + str(start_gate_search_res.pose))
    yield align_for_dummies(sub_singleton, start_gate_search_res)
    fprint("YOLO")

    while(FOUND_START_GATE):
        start_gate_find(sub_singleton)
        yield sub_singleton.move.forward(.3).zero_roll_and_pitch().go(speed=SPEED)

    fprint("Finished")


@txros.util.cancellableInlineCallbacks
def start_gate_find(sub_singleton):
    global FOUND_START_GATE
    start_gate_search = yield sub_singleton.nh.get_service_client('/vision/start_gate/pose', VisionRequest2D)
    start_gate_search_res = yield start_gate_search(VisionRequest2DRequest(target_name=''))
    FOUND_START_GATE = start_gate_search_res.found

@txros.util.cancellableInlineCallbacks
def search_pattern(sub_singleton):
    if not FOUND_START_GATE:
        fprint("Moving up .3", msg_color="yellow")
        yield sub_singleton.move.up(.3).zero_roll_and_pitch().go(speed=SPEED)
        fprint("Moving down .3", msg_color="yellow")
        yield sub_singleton.move.down(.3).zero_roll_and_pitch().go(speed=SPEED)
    if not FOUND_START_GATE:
        fprint("Moving left .3", msg_color="yellow")
        yield sub_singleton.move.left(.3).zero_roll_and_pitch().go(speed=SPEED)
        fprint("Moving right .3", msg_color="yellow")
        yield sub_singleton.move.right(.3).zero_roll_and_pitch().go(speed=SPEED)

    # odom_sub = nh.subscribe('/odom', Odometry)    

    # odom_sub = nh.subscribe('/odom', Odometry)    
    # fprint(odom_sub.pose)


@txros.util.cancellableInlineCallbacks
def align_for_dummies(sub_singleton, start_gate_search_res):
    error_tolerance = 0.1
    deviation = ((start_gate_search_res.pose.x - start_gate_search_res.camera_info.width/2)/start_gate_search_res.camera_info.width,(start_gate_search_res.pose.y - start_gate_search_res.camera_info.height/2)/start_gate_search_res.camera_info.height)
       
    fprint("Alignment -- Deviation: " + str(deviation), msg_color="cyan")
    while(np.abs(deviation[0]) > error_tolerance):
        fprint("Moving right " + str(.1*np.sign(deviation[0])), msg_color="yellow")
        yield sub_singleton.move.right(.1*np.sign(deviation[0])).zero_roll_and_pitch().go(speed=SPEED)
    while(np.abs(deviation[1]) > error_tolerance):
        fprint("Moving up " + str(.1*np.sign(deviation[1])), msg_color="yellow")
        yield sub_singleton.move.up(.1*np.sign(deviation[1])).zero_roll_and_pitch().go(speed=SPEED)
    yield fprint("Finished Alignment -- Deviation: " + str(deviation), msg_color="cyan")
    # while(True):



# def mission_check():
#     odom_sub = nh.subscribe('/odom', Odometry)
