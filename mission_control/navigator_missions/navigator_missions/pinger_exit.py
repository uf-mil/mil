#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
import mil_tools
from mil_misc_tools.text_effects import fprint
from navigator import Navigator

___author___ = "Kevin Allen"


class PingerExitMission(Navigator):
    OBSERVE_DISTANCE_METERS = 6
    GATE_CROSS_METERS = 7
    FREQ = 35000
    LISTEN_TIME = 10
    MAX_CIRCLE_BUOY_ERROR = 30
    CIRCLE_RADIUS = 8

    @txros.util.cancellableInlineCallbacks
    def get_objects(self):
        """Get position of 3 gates from database"""
        gate_1 = yield self.database_query("Gate_1")
        assert gate_1.found, "Gate 1 Not found"
        gate_1_pos = mil_tools.rosmsg_to_numpy(gate_1.objects[0].position)[:2]

        gate_2 = yield self.database_query("Gate_2")
        assert gate_2.found, "Gate 2 Not found"
        gate_2_pos = mil_tools.rosmsg_to_numpy(gate_2.objects[0].position)[:2]

        gate_3 = yield self.database_query("Gate_3")
        assert gate_3.found, "Gate 3 Not found"
        gate_3_pos = mil_tools.rosmsg_to_numpy(gate_3.objects[0].position)[:2]

        self.gate_poses = np.array([gate_1_pos, gate_2_pos, gate_3_pos])

    @txros.util.cancellableInlineCallbacks
    def set_side(self):
        """Set 2 points to observe the pinger from, in front of gates 1 and 3"""
        self.get_gate_perp()
        # Make sure they are actually in a line
        if np.isnan(self.g_perp[0]) or np.isnan(self.g_perp[1]):
            raise Exception("Gates are not in a line")
        neg = yield self.mission_params["pinger_negate"].get()
        self.negate = not neg

    @txros.util.cancellableInlineCallbacks
    def go_thru_gate(self):
        """Move to the points needed to go through the correct gate"""
        self.get_gate_thru_points()
        yield self.move.set_position(self.gate_thru_points[0]).look_at(self.gate_thru_points[1]).go()
        yield self.move.set_position(self.gate_thru_points[1]).go()
        # for p in self.gate_thru_points:
        #    yield self.move.set_position(p).go(initial_plan_time=5)

    def get_gate_perp(self):
        """Calculate a perpendicular to the line formed by the three gates"""
        delta_g = self.gate_poses[2] - self.gate_poses[0]
        rot_right = np.array([[0, -1], [1, 0]])
        g_perp = rot_right.dot(delta_g)
        g_perp = g_perp / np.linalg.norm(g_perp)
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
    def run(self, parameters):
        fprint("PINGER EXIT: Starting", msg_color='green')
        self.gate_index = yield self.mission_params["acoustic_pinger_active_index"].get()
        self.gate_index = self.gate_index - 1
        yield self.get_objects()
        yield self.set_side()
        self.get_gate_thru_points()
        yield self.go_thru_gate()
        fprint("PINGER EXIT: Done", msg_color='green')
