#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from mil_tools import rosmsg_to_numpy
from std_srvs.srv import SetBoolRequest
from mil_tools import quaternion_matrix

___author___ = "Kevin Allen"


class VrxNavigation(Vrx):
    def __init__(self, *args, **kwargs):
        super(VrxNavigation, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def inspect_object(self, position):
        # Go in front of the object, looking directly at it
        yield self.move.look_at(position).set_position(position).backward(8.0).go()
        yield self.nh.sleep(2.)

    @staticmethod
    def get_index_of_type(objects, classifications):
        if type(classifications) == str:
            classifications = [classifications]
        return next(i for i, o in enumerate(objects) if o.labeled_classification in classifications)

    @staticmethod
    def get_gate(one, two, position):
        one = one[:2]
        two = two[:2]
        position = position[:2]
        delta = (one - two)[:2]
        rot_right= np.array([[0, -1], [1, 0]], dtype=np.float)
        perp_vec = rot_right.dot(delta)
        perp_vec = perp_vec / np.linalg.norm(perp_vec)
        center = (one + two) / 2.0
        distances = np.array([
            np.linalg.norm((center + perp_vec) - position),
            np.linalg.norm((center - perp_vec) - position)
        ])
        if np.argmin(distances) == 0:
            perp_vec = -perp_vec
        return np.array([center[0], center[1], 0.]), np.array([perp_vec[0], perp_vec[1], 0.])

    @txros.util.cancellableInlineCallbacks
    def approach_gate(self, one, two, position):
        center, vec = self.get_gate(one, two, position)
        position = center - (vec * 5.0)
        yield self.move.set_position(position).look_at(center).go()

    @txros.util.cancellableInlineCallbacks
    def get_closest_forward(self):
        objects = (yield self.database_query(name='all')).objects
        positions = [rosmsg_to_numpy(obj.pose.position) for obj in objects]
        pose = (yield self.tx_pose)
        # CURRENTLY NOT WORKING
        p = pose[0]
        q_mat = quaternion_matrix(pose[1])
        x_positions_local = [(q_mat.dot(position - p))[0] for position in positions] 
        print x_positions_local 

    @txros.util.cancellableInlineCallbacks
    def prepare_to_enter(self):
        closest = []
        while len(closest) < 2:
            closest, closest_positions = yield self.get_sorted_objects('all', 2)
        if closest[0].labeled_classification == 'UNKNOWN':
           yield self.inspect_object(closest_positions[0]) 
        if closest[1].labeled_classification == 'UNKNOWN':
           yield self.inspect_object(closest_positions[1]) 
        white = closest_positions[self.get_index_of_type(closest, 'surmark46104')]
        red = closest_positions[self.get_index_of_type(closest, ('red_totem', 'surmark950410'))]
        position = (yield self.tx_pose)[0]
        yield self.approach_gate(white, red, position)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        print (yield self.get_closest_forward())
        yield self.set_vrx_classifier_enabled(SetBoolRequest(data=True))
        yield self.prepare_to_enter()
