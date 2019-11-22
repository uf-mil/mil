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
    def go_thru_gate(self, gate, BEFORE=5.0, AFTER=4.0):
        center, vec = gate
        before_position = center - (vec * BEFORE)
        after_position = center + (vec * AFTER)
        yield self.move.set_position(before_position).look_at(center).go()
        if AFTER > 0:
            yield self.move.look_at(after_position).set_position(after_position).go()

    @txros.util.cancellableInlineCallbacks
    def get_objects_of_interest(self, pose):
        p = pose[0]
        q_mat = quaternion_matrix(pose[1])
        objects = (yield self.database_query(name='all')).objects
        positions = np.array([rosmsg_to_numpy(obj.pose.position) for obj in objects])
        positions_local = np.array([(q_mat.T.dot(position - p)) for position in positions])
        positions_local_x = np.array(positions_local[:, 0])
        forward_indicies = np.argwhere(positions_local_x > 1.0).flatten()
        distances = np.linalg.norm(positions_local[forward_indicies], axis=1)
        objects_of_interest_indicies = forward_indicies[np.argsort(distances).flatten()].tolist()
        ret_objects = [objects[i] for i in objects_of_interest_indicies]
        defer.returnValue((ret_objects, positions[objects_of_interest_indicies], positions_local[objects_of_interest_indicies]))

    @txros.util.cancellableInlineCallbacks
    def do_next_gate(self):
        pose = (yield self.tx_pose)
        p = pose[0]
        # CURRENTLY NOT WORKING
        investigated = set()
        while True:
            objects_of_interest, positions, positions_local = yield self.get_objects_of_interest(pose)
            if len(objects_of_interest) < 2:
                raise Exception('we done')
            try:
                left_index = self.get_index_of_type(objects_of_interest, ('surmark950400', 'green_totem', 'blue_totem'))
                right_index = self.get_index_of_type(objects_of_interest, ('red_totem', 'surmark950410'))
                break
            except StopIteration:
                pass
            explored = False
            for i in range(len(objects_of_interest)):
                if objects_of_interest[i].id not in investigated and objects_of_interest[i].labeled_classification == 'UNKNOWN':
                    yield self.inspect_object(positions[i])
                    investigated.add(objects_of_interest[i].id)
                    explored = True
                    break
            if explored:
                continue
            self.send_feedback('!!! NO MORE OBJECTS TO INVESTIGATE, CHOOSING TWO RANDOM BOIS')
            left_index = np.argmax(positions_local[:, 1])
            right_index = np.argmin(positions_local[:, 1])
            break
        end = objects_of_interest[left_index].labeled_classification == 'blue_totem'
        self.send_feedback('done' if end else 'not done, bro')
        left_position = positions[left_index]
        right_position = positions[right_index]
        gate = self.get_gate(left_position, right_position, p)
        yield self.go_thru_gate(gate)
        defer.returnValue(end)

    @txros.util.cancellableInlineCallbacks
    def prepare_to_enter(self):
        closest = []
        while len(closest) < 2:
            res = yield self.get_sorted_objects('all', 2, throw=False)
            if res is None:
                continue
            closest, closest_positions = res
        if closest[0].labeled_classification == 'UNKNOWN':
           yield self.inspect_object(closest_positions[0]) 
        if closest[1].labeled_classification == 'UNKNOWN':
           yield self.inspect_object(closest_positions[1]) 
        white = closest_positions[self.get_index_of_type(closest, 'surmark46104')]
        red = closest_positions[self.get_index_of_type(closest, ('red_totem', 'surmark950410'))]
        position = (yield self.tx_pose)[0]
        gate = self.get_gate(white, red, position)
        yield self.go_thru_gate(gate, AFTER=-2)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        '''
        TODO: check for new objects in background, cancel move
              somefucking how handle case where gates litteraly loop back and head the other direction
        '''
        yield self.set_vrx_classifier_enabled(SetBoolRequest(data=True))
        yield self.prepare_to_enter()
        yield self.wait_for_task_such_that(lambda task: task.state =='running')
        yield self.move.forward(4.0).go()
        while not (yield self.do_next_gate()):
            pass
        yield self.move.forward(10).go()
        yield self.set_vrx_classifier_enabled(SetBoolRequest(data=False))

