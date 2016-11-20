#!/usr/bin/env python
from __future__ import division
import txros
import std_srvs.srv
import numpy as np
from navigator_msgs.srv import FindPinger, FindPingerRequest, SetFrequency, SetFrequencyRequest
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool, SetBoolRequest
import navigator_tools
from navigator_tools import fprint

___author___ = "Kevin Allen"

class PingerMission:
    OBSERVE_DISTANCE_METERS = 12
    GATE_CROSS_METERS = 10
    FREQ = 35000
    LISTEN_TIME = 15

    def __init__(self, navigator):
        self.navigator = navigator
        self.listen_client = self.navigator.nh.get_service_client('/hydrophones/set_listen', SetBool)
        self.pinger_client = self.navigator.nh.get_service_client('/hydrophones/find_pinger', FindPinger)
        self.reset_client = self.navigator.nh.get_service_client('/hydrophones/set_freq', SetFrequency)
        self.negate = False

    def reset_freq(self):
        return self.reset_client(SetFrequencyRequest(frequency=self.FREQ))

    def start_listen(self):
        return self.listen_client(SetBoolRequest(data=True))

    def stop_listen(self):
        return self.listen_client(SetBoolRequest(data=False))

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

    @txros.util.cancellableInlineCallbacks
    def get_observation_poses(self):
        """Set 2 points to observe the pinger from, in front of gates 1 and 3"""
        self.get_gate_perp()
        pose = (yield self.navigator.tx_pose)[0][:2]
        distance_test = np.array([np.linalg.norm(pose - (self.gate_poses[0] + self.OBSERVE_DISTANCE_METERS * self.g_perp)),
                                  np.linalg.norm(pose - (self.gate_poses[0] - self.OBSERVE_DISTANCE_METERS * self.g_perp))])
        if np.argmin(distance_test) == 1:
            self.negate = True
        if self.negate:
            self.observation_points = (np.append((self.gate_poses[0] - self.OBSERVE_DISTANCE_METERS * self.g_perp), 0),
                                       np.append((self.gate_poses[2] - self.OBSERVE_DISTANCE_METERS * self.g_perp), 0))
        else:
            self.observation_points = (np.append((self.gate_poses[0] + self.OBSERVE_DISTANCE_METERS * self.g_perp), 0),
                                       np.append((self.gate_poses[2] + self.OBSERVE_DISTANCE_METERS * self.g_perp), 0))
        self.look_at_points = (np.append(self.gate_poses[0], 0), np.append(self.gate_poses[2], 0))

    @txros.util.cancellableInlineCallbacks
    def search_samples(self):
        """Move to each observation point and listen to the pinger while sitting still"""
        yield self.get_observation_poses()
        for i,p in enumerate(self.observation_points):
            yield self.stop_listen()
            yield self.navigator.move.set_position(p).look_at(self.look_at_points[i]).go(move_type="skid")
            yield self.start_listen()
            fprint("PINGER: Listening To Pinger at point {}".format(p), msg_color='green')
            yield self.navigator.nh.sleep(self.LISTEN_TIME)
        yield self.stop_listen()

    @txros.util.cancellableInlineCallbacks
    def get_pinger_pose(self):
        """Query pinger perception for the location of the pinger after observing"""
        res = yield self.pinger_client(FindPingerRequest())
        pinger_pose = res.pinger_position
        self.pinger_pose = navigator_tools.rosmsg_to_numpy(pinger_pose)[:2]
        self.distances = np.array([np.linalg.norm(self.pinger_pose - self.gate_poses[0]),
                                   np.linalg.norm(self.pinger_pose - self.gate_poses[1]),
                                   np.linalg.norm(self.pinger_pose - self.gate_poses[2])])

    @txros.util.cancellableInlineCallbacks
    def go_thru_gate(self):
        """Move to the points needed to go through the correct gate"""
        self.gate_index = np.argmin(self.distances)
        self.get_gate_thru_points()
        for p in self.gate_thru_points:
            yield self.navigator.move.set_position(p).go(initial_plan_time=5)

    @txros.util.cancellableInlineCallbacks
    def set_active_pinger(self):
        """Set the paramter for the active pinger identified for use in other mission"""
        if self.gate_index == 0:
            yield self.navigator.mission_params["acoustic_pinger_active"].set("RED-WHITE")
        elif self.gate_index == 1:
            yield self.navigator.mission_params["acoustic_pinger_active"].set("WHITE-WHITE")
        elif self.gate_index == 2:
            yield self.navigator.mission_params["acoustic_pinger_active"].set("WHITE-GREEN")

    def get_gate_perp(self):
        """Calculate a perpendicular to the line formed by the three gates"""
        delta_g = self.gate_poses[2] - self.gate_poses[0]
        rot_right = np.array([[0, -1], [1, 0]])
        g_perp = rot_right.dot(delta_g)
        g_perp = g_perp /  np.linalg.norm(g_perp)
        self.g_perp = g_perp

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
        fprint("PINGER: Starting Pinger Mission", msg_color='green')
        fprint("PINGER: Reseting Frequency", msg_color='green')
        yield self.reset_freq()
        fprint("PINGER: Getting database objects", msg_color='green')
        yield self.get_objects()
        fprint("PINGER: Calculating observation points", msg_color='green') 
        yield self.get_observation_poses()
        fprint("PINGER: Searching each sample", msg_color='green') 
        yield self.search_samples()
        fprint("PINGER: Getting pinger position", msg_color='green') 
        yield self.get_pinger_pose()
        fprint("PINGER: Going through gate", msg_color='green') 
        yield self.go_thru_gate()
        fprint("PINGER: Setting active pinger parameter", msg_color='green') 
        yield self.set_active_pinger()
        fprint("PINGER: Ended Pinger Mission", msg_color='green') 

@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    mission = PingerMission(navigator)
    yield mission.run()

