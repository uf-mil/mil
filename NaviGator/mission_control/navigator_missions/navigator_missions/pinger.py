#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from navigator_msgs.srv import FindPinger, FindPingerRequest, SetFrequency, SetFrequencyRequest
from std_srvs.srv import SetBool, SetBoolRequest
import mil_tools
from visualization_msgs.msg import Marker, MarkerArray
from mil_misc_tools.text_effects import fprint
from navigator import Navigator

___author___ = "Kevin Allen"


class PingerMission(Navigator):
    OBSERVE_DISTANCE_METERS = 7
    GATE_CROSS_METERS = 8
    FREQ = 27000
    LISTEN_TIME = 10
    MAX_CIRCLE_BUOY_ERROR = 30
    CIRCLE_RADIUS = 8
    USE_CLOSE_POINTS = False

    def __init__(self):
        super(PingerMission, self).__init__()
        self.negate = False
        self.markers = MarkerArray()
        self.last_id = 0
        self.circle_totem = None
        self.color_wrong = False

    @classmethod
    def init(cls):
        cls.listen_client = cls.nh.get_service_client('/hydrophones/set_listen', SetBool)
        cls.pinger_client = cls.nh.get_service_client('/hydrophones/find_pinger', FindPinger)
        cls.reset_client = cls.nh.get_service_client('/hydrophones/set_freq', SetFrequency)
        cls.marker_pub = cls.nh.advertise("/pinger/debug_marker", MarkerArray)

    def reset_freq(self):
        return self.reset_client(SetFrequencyRequest(frequency=self.FREQ))

    def start_listen(self):
        return self.listen_client(SetBoolRequest(data=True))

    def stop_listen(self):
        return self.listen_client(SetBoolRequest(data=False))

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
    def get_observation_poses(self):
        """Set 2 points to observe the pinger from, in front of gates 1 and 3"""
        self.get_gate_perp()
        # Make sure they are actually in a line
        if np.isnan(self.g_perp[0]) or np.isnan(self.g_perp[1]):
            raise Exception("Gates are not in a line")
            return

        pose = self.pose[0][:2]
        distance_test = np.array([
            np.linalg.norm(pose - (self.gate_poses[0] + self.OBSERVE_DISTANCE_METERS * self.g_perp)),
            np.linalg.norm(pose - (self.gate_poses[0] - self.OBSERVE_DISTANCE_METERS * self.g_perp))])
        if np.argmin(distance_test) == 1:
            self.negate = True
        yield self.mission_params["pinger_negate"].set(self.negate)
        if self.USE_CLOSE_POINTS:
            dis = (self.gate_poses[2] - self.gate_poses[0]) / 3.0
            branch_pt0 = self.gate_poses[0] + dis
            branch_pt1 = self.gate_poses[2] - dis
            if self.negate:
                self.observation_points = (np.append((branch_pt0 - self.OBSERVE_DISTANCE_METERS * self.g_perp), 0),
                                           np.append((branch_pt1 - self.OBSERVE_DISTANCE_METERS * self.g_perp), 0))
            else:
                self.observation_points = (np.append((branch_pt0 + self.OBSERVE_DISTANCE_METERS * self.g_perp), 0),
                                           np.append((branch_pt1 + self.OBSERVE_DISTANCE_METERS * self.g_perp), 0))
            self.look_at_points = (np.append(branch_pt0, 0), np.append(branch_pt1, 0))
        else:
            if self.negate:
                self.observation_points = (
                    np.append((self.gate_poses[0] - self.OBSERVE_DISTANCE_METERS * self.g_perp), 0),
                    np.append((self.gate_poses[2] - self.OBSERVE_DISTANCE_METERS * self.g_perp), 0))
            else:
                self.observation_points = (
                    np.append((self.gate_poses[0] + self.OBSERVE_DISTANCE_METERS * self.g_perp), 0),
                    np.append((self.gate_poses[2] + self.OBSERVE_DISTANCE_METERS * self.g_perp), 0))
            self.look_at_points = (np.append(self.gate_poses[0], 0), np.append(self.gate_poses[2], 0))

    @txros.util.cancellableInlineCallbacks
    def search_samples(self):
        """Move to each observation point and listen to the pinger while sitting still"""
        yield self.get_observation_poses()
        for i, p in enumerate(self.observation_points):
            yield self.stop_listen()
            yield self.move.set_position(p).look_at(self.look_at_points[i]).go()
            yield self.start_listen()
            fprint("PINGER: Listening To Pinger at point {}".format(p), msg_color='green')
            yield self.nh.sleep(self.LISTEN_TIME)
        yield self.stop_listen()

    @txros.util.cancellableInlineCallbacks
    def get_pinger_pose(self):
        """Query pinger perception for the location of the pinger after observing"""
        res = yield self.pinger_client(FindPingerRequest())
        if res.pinger_position.x == 0:
            self.pinger_pose = self.gate_poses[1]
        pinger_pose = res.pinger_position
        self.pinger_pose = mil_tools.rosmsg_to_numpy(pinger_pose)[:2]
        self.distances = np.array([np.linalg.norm(self.pinger_pose - self.gate_poses[0]),
                                   np.linalg.norm(self.pinger_pose - self.gate_poses[1]),
                                   np.linalg.norm(self.pinger_pose - self.gate_poses[2])])

    @txros.util.cancellableInlineCallbacks
    def go_thru_gate(self):
        """Move to the points needed to go through the correct gate"""
        self.gate_index = np.argmin(self.distances)
        self.get_gate_thru_points()
        for p in self.gate_thru_points:
            yield self.move.set_position(p).go(initial_plan_time=5)

    def new_marker(self, ns="/debug", frame="enu", time=None, type=Marker.CUBE,
                   position=(0, 0, 0), orientation=(0, 0, 0, 1), color=(1, 0, 0)):
        marker = Marker()
        marker.ns = ns
        if time is not None:
            marker.header.stamp = time
        marker.header.frame_id = frame
        marker.type = type
        marker.action = marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.id = self.last_id
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        self.last_id += 1
        self.markers.markers.append(marker)

    @txros.util.cancellableInlineCallbacks
    def get_colored_buoys(self):
        estimated_3_endbuoy = self.gate_poses[2] + (self.g_line * 5.0)
        estimated_1_endbuoy = self.gate_poses[0] - (self.g_line * 5.0)
        if self.negate:
            # should check on correct side (negate bool)
            estimated_circle_buoy = self.gate_poses[1] + (self.g_perp * 30.0)
        else:
            # should check on correct side (negate bool)
            estimated_circle_buoy = self.gate_poses[1] - (self.g_perp * 30.0)

        cur_time = yield self.nh.get_time()
        self.new_marker(position=np.append(estimated_3_endbuoy, 0), color=(1, 1, 1), time=cur_time)
        self.new_marker(position=np.append(estimated_circle_buoy, 0), time=cur_time)
        self.new_marker(position=np.append(estimated_1_endbuoy, 0), color=(1, 1, 1), time=cur_time)

        totems = yield self.database_query("totem", raise_exception=False)
        if totems.found:
            sorted_1 = sorted(totems.objects, key=lambda t: np.linalg.norm(
                estimated_1_endbuoy - mil_tools.rosmsg_to_numpy(t.position)[:2]))
            sorted_3 = sorted(totems.objects, key=lambda t: np.linalg.norm(
                estimated_3_endbuoy - mil_tools.rosmsg_to_numpy(t.position)[:2]))

            sorted_circle = sorted(totems.objects, key=lambda t: abs(np.linalg.norm(
                estimated_circle_buoy - mil_tools.rosmsg_to_numpy(t.position)[:2])))
            self.new_marker(position=mil_tools.rosmsg_to_numpy(
                sorted_circle[0].position), color=(1, 1, 1), time=cur_time)
            circle_error = np.linalg.norm(
                mil_tools.rosmsg_to_numpy(sorted_circle[0].position)[:2] - estimated_circle_buoy)
            if circle_error < self.MAX_CIRCLE_BUOY_ERROR:
                self.circle_totem = mil_tools.rosmsg_to_numpy(sorted_circle[0].position)
                fprint("PINGER: found buoy to circle at {}".format(self.circle_totem), msg_color='green')
            else:
                fprint("PINGER: circle buoy is too far from where it should be", msg_color='red')

            if sorted_3[0].color.r > 0.9:
                self.color_wrong = True
                color_3 = "RED"
                self.new_marker(position=mil_tools.rosmsg_to_numpy(
                    sorted_3[0].position), color=(1, 0, 0), time=cur_time)
            elif sorted_3[0].color.g > 0.9:
                color_3 = "GREEN"
                self.new_marker(position=mil_tools.rosmsg_to_numpy(
                    sorted_3[0].position), color=(0, 1, 0), time=cur_time)
            else:
                color_3 = "UNKNOWN"
                self.new_marker(position=mil_tools.rosmsg_to_numpy(
                    sorted_3[0].position), color=(1, 1, 1), time=cur_time)
            if sorted_1[0].color.r > 0.9:
                color_1 = "RED"
                self.new_marker(position=mil_tools.rosmsg_to_numpy(
                    sorted_1[0].position), color=(1, 0, 0), time=cur_time)
            elif sorted_1[0].color.g > 0.9:
                self.color_wrong = True
                color_1 = "GREEN"
                self.new_marker(position=mil_tools.rosmsg_to_numpy(
                    sorted_1[0].position), color=(0, 1, 0), time=cur_time)
            else:
                color_1 = "UNKNOWN"
                self.new_marker(position=mil_tools.rosmsg_to_numpy(
                    sorted_1[0].position), color=(1, 1, 1), time=cur_time)

            if int(self.gate_index) == 0:
                if color_1 == "RED":
                    active_colors = "RED-WHITE"
                elif color_1 == "GREEN":
                    active_colors = "WHITE-GREEN"
                else:
                    active_colors = "UNKNOWN"
            elif int(self.gate_index) == 2:
                if color_3 == "RED":
                    active_colors = "RED-WHITE"
                elif color_3 == "GREEN":
                    active_colors = "WHITE-GREEN"
                else:
                    active_colors = "UNKNOWN"
            else:
                active_colors = "WHITE-WHITE"

            if active_colors != "UNKNOWN":
                fprint("PINGER: setting active pinger colors to {}".format(active_colors), msg_color='green')
                yield self.mission_params["acoustic_pinger_active"].set(active_colors)
            else:
                fprint("PINGER: cannot determine gate colors".format(sorted_3[0].color), msg_color='red')
            #  fprint("PINGER: gate 3 color {}".format(sorted_3[0].color), msg_color='blue')
            #  fprint("PINGER: gate 1 color {}".format(sorted_1[0].color), msg_color='blue')
        else:
            fprint("PINGER: no totems found", msg_color='red')
        yield self.marker_pub.publish(self.markers)

    @txros.util.cancellableInlineCallbacks
    def set_active_pinger(self):
        """Set the paramter for the active pinger identified for use in other mission"""
        fprint("PINGER: setting active pinger to Gate_{}".format(int(self.gate_index) + 1), msg_color='green')
        yield self.get_colored_buoys()
        if self.color_wrong and self.gate_index == 2:
            yield self.mission_params["acoustic_pinger_active_index_correct"].set(1)
        elif self.color_wrong and self.gate_index == 0:
            yield self.mission_params["acoustic_pinger_active_index_correct"].set(3)
        else:
            yield self.mission_params["acoustic_pinger_active_index_correct"].set(int(self.gate_index) + 1)

        yield self.mission_params["acoustic_pinger_active_index"].set(int(self.gate_index) + 1)
        # yield self.get_colored_buoys()

    @txros.util.cancellableInlineCallbacks
    def circle_buoy(self):
        if self.circle_totem is not None:
            # Circle around totem
            yield self.move.look_at(self.circle_totem).set_position(self.circle_totem).backward(8)\
                           .yaw_left(90, unit='deg').go()
            yield self.move.circle_point(self.circle_totem).go()

            (first, last) = reversed(self.gate_thru_points)
            yield self.move.set_position(first).look_at(last).go(initial_plan_time=5, move_type="drive")
            yield self.move.set_position(last).go(initial_plan_time=5, move_type="drive")

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
    def cleanup(self):
        yield self.mission_params["acoustic_pinger_active_index"].set(1)
        yield self.mission_params["acoustic_pinger_active_index_correct"].set(1)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        fprint("PINGER: Starting Pinger Mission", msg_color='green')
        fprint("PINGER: Reseting Frequency", msg_color='green')
        yield self.reset_freq()
        fprint("PINGER: Getting database objects", msg_color='green')
        yield self.get_objects()
        fprint("PINGER: Calculating observation points", msg_color='green')
        fprint("PINGER: Searching each sample", msg_color='green')
        yield self.search_samples()
        fprint("PINGER: Getting pinger position", msg_color='green')
        yield self.get_pinger_pose()
        fprint("PINGER: Going through gate", msg_color='green')
        yield self.go_thru_gate()
        yield self.set_active_pinger()
        #  yield self.circle_buoy()
        fprint("PINGER: Ended Pinger Mission", msg_color='green')
