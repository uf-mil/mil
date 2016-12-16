#!/usr/bin/env python
from __future__ import division
import txros
import std_srvs.srv
import numpy as np
from navigator_msgs.srv import FindPinger, FindPingerRequest, SetFrequency, SetFrequencyRequest
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool, SetBoolRequest
import navigator_tools
from visualization_msgs.msg import Marker, MarkerArray
from navigator_tools import fprint
import rospy

___author___ = "Kevin Allen"

class PingerExitMission:
    OBSERVE_DISTANCE_METERS = 5
    GATE_CROSS_METERS = 5
    FREQ = 35000
    LISTEN_TIME = 10
    MAX_CIRCLE_BUOY_ERROR = 30
    CIRCLE_RADIUS = 8

    def __init__(self, navigator):
        self.navigator = navigator

    @txros.util.cancellableInlineCallbacks
    def get_objects(self):
        """Get position of 3 gates from database"""
        gate_1 = yield self.navigator.database_query("Gate_1")
        assert gate_1.found, "Gate 1 Not found"
        gate_1_pos = navigator_tools.rosmsg_to_numpy(gate_1.objects[0].position)[:2]

        gate_2 = yield self.navigator.database_query("Gate_2")
        assert gate_2.found, "Gate 2 Not found"
        gate_2_pos = navigator_tools.rosmsg_to_numpy(gate_2.objects[0].position)[:2]

        gate_3 = yield self.navigator.database_query("Gate_3")
        assert gate_3.found, "Gate 3 Not found"
        gate_3_pos = navigator_tools.rosmsg_to_numpy(gate_3.objects[0].position)[:2]

        self.gate_poses = np.array([gate_1_pos, gate_2_pos, gate_3_pos])

    def set_side(self):
        """Set 2 points to observe the pinger from, in front of gates 1 and 3"""
        self.get_gate_perp()
        #Make sure they are actually in a line
        if np.isnan(self.g_perp[0]) or np.isnan(self.g_perp[1]):
            raise Exception("Gates are not in a line")
        pose = self.navigator.pose[0][:2]
        distance_test = np.array([np.linalg.norm(pose - (self.gate_poses[self.gate_index] + self.OBSERVE_DISTANCE_METERS * self.g_perp)),
                                  np.linalg.norm(pose - (self.gate_poses[self.gate_index] - self.OBSERVE_DISTANCE_METERS * self.g_perp))])
        if np.argmin(distance_test) == 1:
            self.negate = True

    @txros.util.cancellableInlineCallbacks
    def go_thru_gate(self):
        """Move to the points needed to go through the correct gate"""
        self.gate_index = np.argmin(self.distances)
        self.get_gate_thru_points()
        for p in self.gate_thru_points:
            yield self.navigator.move.set_position(p).go(initial_plan_time=5)

    def get_gate_perp(self):
        """Calculate a perpendicular to the line formed by the three gates"""
        delta_g = self.gate_poses[2] - self.gate_poses[0]
        rot_right = np.array([[0, -1], [1, 0]])
        g_perp = rot_right.dot(delta_g)
        g_perp = g_perp /  np.linalg.norm(g_perp)
        self.g_perp = g_perp
        self.g_line = delta_g / np.linalg.norm(delta_g)

    def get_gate_thru_points(self):
        """Set points needed to cross through correct gate"""
        if self.negate:
            pose1 = self.gate_poses[self.gate_index] - self.GATE_CROSS_METERS * self.g_perp
            pose2 = self.gate_poses[self.gate_index] + self.GATE_CROSS_METERS * self.g_perp
        else:
            pose1 = self.gate_poses[self.gate_index] + self.GATE_CROSS_METERS * self.g_perp
            pose2 = self.gate_poses[self.gate_index] - self.GATE_CROSS_METERS * self.g_perp
        self.gate_thru_points = (np.append(pose1, 0), np.append(pose2, 0))

    @txros.util.cancellableInlineCallbacks
    def run(self):
        fprint("PINGER EXIT: Starting", msg_color='green') 
        self.gate_index = yield navigator.mission_params["acoustic_pinger_active_index"].get()
        yield self.get_objects()
        yield self.set_side()
        self.get_gate_thru_points()
        yield self.go_thru_gate()
        fprint("PINGER EXIT: Done", msg_color='green') 

@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    mission = PingerExitMission(navigator)
    yield mission.run()


