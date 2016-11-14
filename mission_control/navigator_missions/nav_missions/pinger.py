#!/usr/bin/env python
from __future__ import division
import txros
import std_srvs.srv
import numpy as np
import tf
import tf.transformations as trns
from navigator_msgs.msg import ShooterDoAction, ShooterDoActionGoal
from navigator_msgs.srv import CameraToLidarTransform, CameraToLidarTransformRequest, FindPinger, FindPingerRequest, SetFrequency, SetFrequencyRequest
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool, SetBoolRequest
from twisted.internet import defer
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker,MarkerArray
import navigator_tools

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
        self.markers_pub = self.navigator.nh.advertise("/pinger/debug_marker",MarkerArray)
        self.negate = False

    @txros.util.cancellableInlineCallbacks
    def reset_freq(self):
        yield self.reset_client(SetFrequencyRequest(frequency=self.FREQ))

    #@txros.util.cancellableInlineCallbacks
    def start_listen(self):
        return self.listen_client(SetBoolRequest(data=True))

    #@txros.util.cancellableInlineCallbacks
    def stop_listen(self):
        return self.listen_client(SetBoolRequest(data=False))

    @txros.util.cancellableInlineCallbacks
    def get_objects(self):
        self.gate_1 = yield self.navigator.database_query("Gate_1")
        assert self.gate_1.found, "Not found"
        self.gate_1_pos = navigator_tools.rosmsg_to_numpy(self.gate_1.objects[0].position)[:2]
        self.gate_2 = yield self.navigator.database_query("Gate_2")
        assert self.gate_2.found, "Not found"
        self.gate_2_pos = navigator_tools.rosmsg_to_numpy(self.gate_2.objects[0].position)[:2]
        self.gate_3 = yield self.navigator.database_query("Gate_3")
        assert self.gate_3.found, "Not found"
        self.gate_3_pos = navigator_tools.rosmsg_to_numpy(self.gate_3.objects[0].position)[:2]

    def get_observation_poses(self):
        self._get_perp(self.gate_1_pos, self.gate_3_pos)

        pose = yield self.navigator.tx_pose()
        self.dist_sign_pos = np.linalg.norm(pose[:2] - (self.gate_1_pos + self.OBSERVE_DISTANCE_METERS * self.g_perp) )
        self.dist_sign_neg = np.linalg.norm(pose[:2] - (self.gate_1_pos - self.OBSERVE_DISTANCE_METERS * self.g_perp) )
        if np.argmin( np.array( [self.dist_sign_pos,self.dist_sign_neg] ) ) == 1:
            self.negate = True
        if self.negate:
            self.observation_points =  ( np.append( (self.gate_1_pos - self.observer_sign * self.OBSERVE_DISTANCE_METERS * self.g_perp), 0),
                                         np.append( (self.gate_3_pos - self.OBSERVE_DISTANCE_METERS * self.g_perp), 0) )
        else:
             self.observation_points =  ( np.append( (self.gate_1_pos + self.observer_sign * self.OBSERVE_DISTANCE_METERS * self.g_perp), 0),
                                         np.append( (self.gate_3_pos + self.OBSERVE_DISTANCE_METERS * self.g_perp), 0) )
        self.look_at_points = (np.append(self.gate_1_pos,0), np.append(self.gate_3_pos,0))
        self.observation_markers()

    @txros.util.cancellableInlineCallbacks
    def search_samples(self):
        for i,p in enumerate(self.observation_points):
          yield self.stop_listen()
          yield self.navigator.move.set_position(p).look_at(self.look_at_points[i]).go()
          yield self.start_listen()
          print "Listening at ",p
          yield self.navigator.nh.sleep(self.LISTEN_TIME)
        yield self.stop_listen()

    @txros.util.cancellableInlineCallbacks
    def get_pinger_pose(self):
        res = yield self.pinger_client(FindPingerRequest())
        pinger_pose = res.pinger_position
        self.pinger_pose = navigator_tools.rosmsg_to_numpy(pinger_pose)[:2]
        self.distance1 = np.linalg.norm(self.pinger_pose - self.gate_1_pos) 
        self.distance2 = np.linalg.norm(self.pinger_pose - self.gate_2_pos)
        self.distance3 = np.linalg.norm(self.pinger_pose - self.gate_3_pos)

    @txros.util.cancellableInlineCallbacks
    def go_thru_gate(self):
        self.gate_index = np.argmin( np.array( [self.distance1,self.distance2,self.distance3] ) )
        self.gate_thru_poses  = self._thru_gate_poses( (self.gate_1_pos,self.gate_2_pos,self.gate_3_pos), self.gate_index)
        self.gate_thru_markers()
        for p in self.gate_thru_poses:
            yield self.navigator.move.set_position(p).go(initial_plan_time=5)

    @txros.util.cancellableInlineCallbacks
    def set_active_pinger(self):
      if self.gate_index == 0:
          yield self.navigator.mission_params["acoustic_pinger_active"].set("RED-WHITE")
      elif self.gate_index == 1:
          yield self.navigator.mission_params["acoustic_pinger_active"].set("WHITE-WHITE")
      elif self.gate_index == 2:
          yield self.navigator.mission_params["acoustic_pinger_active"].set("WHITE-GREEN")

    def _get_perp(self,g1,g3):
        assert len(g1) == 2
        assert len(g3) == 2
        delta_g = g3 - g1
        rot_right = np.array([[0, -1], [1, 0]])
        g_perp = rot_right.dot(delta_g)
        g_perp = g_perp /  np.linalg.norm(g_perp)
        self.g_perp = g_perp

    def _thru_gate_poses(self, gatees, index):
        pose2 = None
        if index == 0:
            pose2 = self.gate_1_pos
        elif index == 1:
            pose2 = self.gate_2_pos
        elif index == 2:
            pose2 = self.gate_3_pos
        if self.negate:
            pose1 = pose2 - self.GATE_CROSS_METERS * self.g_perp
            pose3 = pose2 + self.GATE_CROSS_METERS * self.g_perp
        else:
            pose1 = pose2 + self.GATE_CROSS_METERS * self.g_perp
            pose3 = pose2 - self.GATE_CROSS_METERS * self.g_perp   
        return (np.append(pose1,0), np.append(pose3,0))

    @txros.util.cancellableInlineCallbacks
    def observation_markers(self):
        markers = MarkerArray()
        for i, p in enumerate(self.observation_points):
          marker = Marker()
          marker.scale.x = 1;
          marker.scale.y = 1;
          marker.scale.z = 1;
          marker.action = Marker.ADD;
          marker.header.frame_id = "enu"
          marker.header.stamp = self.navigator.nh.get_time()
          marker.pose.position = navigator_tools.numpy_to_point(p)
          marker.type = Marker.SPHERE
          marker.text = "Offset goal"
          marker.ns = "pinger"
          marker.id = 3000 + i
          marker.color.b = 1
          marker.color.a = 1
          markers.markers.append(marker)
        yield self.markers_pub.publish(markers)

    @txros.util.cancellableInlineCallbacks
    def gate_thru_markers(self):
        markers = MarkerArray()
        for i, p in enumerate(self.gate_thru_poses):
          marker = Marker()
          marker.scale.x = 1;
          marker.scale.y = 1;
          marker.scale.z = 1;
          marker.action = Marker.ADD;
          marker.header.frame_id = "enu"
          marker.header.stamp = self.navigator.nh.get_time()
          marker.pose.position = navigator_tools.numpy_to_point(p)
          marker.type = Marker.SPHERE
          marker.text = "Offset goal"
          marker.ns = "pinger"
          marker.id = 4000 + i
          marker.color.r = 1     
          marker.color.a = 1
          markers.markers.append(marker)
        yield self.markers_pub.publish(markers)

    @txros.util.cancellableInlineCallbacks
    def run(self):
        print "Starting pinger"
        print "Reseting Frequency"
        yield self.reset_freq()
        print "Getting database objects"
        yield self.get_objects()
        print "Calculating observation points"
        yield self.get_observation_poses()
        print "Searching each sample"
        yield self.search_samples()
        print "Getting pinger position"
        yield self.get_pinger_pose()
        print "Going through gate"
        yield self.go_thru_gate()
        print "Setting active pinger parameter"
        yield self.set_active_pinger()
        print "Ended Pinger Mission"

@txros.util.cancellableInlineCallbacks
def main(navigator):
    mission = PingerMission(navigator)
    yield mission.run()

