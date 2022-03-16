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
        yield self.move.look_at(position).set_position(position).backward(6.0).go()
        yield self.nh.sleep(5.)

    def get_index_of_type(self, objects, classifications):
        if type(classifications) == str:
            classifications = [classifications]
        return next(i for i, o in enumerate(objects) if o.labeled_classification in classifications and o.id not in self.objects_passed)

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
    def do_next_gate(self):
        pose = yield self.tx_pose
        p = pose[0]
        q_mat = quaternion_matrix(pose[1])

        def filter_and_sort(objects, positions):
            positions_local = np.array([(q_mat.T.dot(position - p)) for position in positions])
            positions_local_x = np.array(positions_local[:, 0])
            forward_indicies = np.argwhere(positions_local_x > 1.0).flatten()
            forward_indicies = np.array([i for i in forward_indicies if objects[i].id not in self.objects_passed])
            distances = np.linalg.norm(positions_local[forward_indicies], axis=1)
            indicies =  forward_indicies[np.argsort(distances).flatten()].tolist()
            # ids = [objects[i].id for i in indicies]
            # self.send_feedback('Im attracted to {}'.format(ids))
            return indicies

        def is_done(objects, positions):
            try:
                left_index = self.get_index_of_type(objects, ('surmark950400', 'green_totem', 'blue_totem'))
                right_index = self.get_index_of_type(objects, ('red_totem', 'surmark950410'))
            except StopIteration:
                return None
            end = objects[left_index].labeled_classification == 'blue_totem'
            return positions[left_index], objects[left_index], positions[right_index], objects[right_index], end

        left, left_obj, right, right_obj, end = yield self.explore_closest_until(is_done, filter_and_sort)
        self.send_feedback('Going through gate of objects {} and {}'.format(left_obj.id, right_obj.id))
        gate = self.get_gate(left, right, p)
        yield self.go_thru_gate(gate)
        self.objects_passed.add(left_obj.id)
        self.objects_passed.add(right_obj.id)
        defer.returnValue(end)

    @txros.util.cancellableInlineCallbacks
    def explore_closest_until(self, is_done, filter_and_sort):
        '''
        @conditon func taking in sorted objects, positions
        @object_filter func filters and sorts
        '''
        move_id_tuple = None
        service_req = None
        dl = None
        investigated = set()
        while True:
            if move_id_tuple is not None:
                if service_req is None:
                    service_req = self.database_query(name='all')
                dl = defer.DeferredList([service_req, move_id_tuple[0]], fireOnOneCallback=True)
                result, index = yield dl

                # Database query sucseeded
                if index == 0:
                    service_req = None
                    objects_msg = result
                    if self.object_classified(objects_msg.objects, move_id_tuple[1]):
                        self.send_feedback('{} identified. Canceling investigation'.format(move_id_tuple[1]))
                        yield dl.cancel()
                        yield move_id_tuple[0].cancel()
                        # yield self.move.forward(0).go()
                        move_id_tuple = None
                # Move succeeded:
                else:
                    self.send_feedback('Investigated {}'.format(move_id_tuple[1]))
                    move_id_tuple = None
            else:
                objects_msg = yield self.database_query(name='all')
            objects = objects_msg.objects
            positions = np.array([rosmsg_to_numpy(obj.pose.position) for obj in objects])
            if len(objects) == 0:
                indicies = []
            else:
                indicies = filter_and_sort(objects, positions)
            if indicies is None or len(indicies) == 0:
                self.send_feedback('No objects')
                continue
            objects = [objects[i] for i in indicies]
            positions = positions[indicies]
            # Exit if done
            ret = is_done(objects, positions)
            if ret is not None:
                if move_id_tuple is not None:
                    self.send_feedback('Condition met. Canceling investigation')
                    yield dl.cancel()
                    yield move_id_tuple[0].cancel()
                    # yield self.move.forward(0).go()
                    move_id_tuple = None
                defer.returnValue(ret)

            if move_id_tuple is not None:
                continue

            self.send_feedback('ALREADY INVEST {}'.format(investigated))

            # Explore the next one
            for i in xrange(len(objects)):
                if objects[i].labeled_classification == 'UNKNOWN' and objects[i].id not in investigated:
                    self.send_feedback('Investingating {}'.format(objects[i].id))
                    investigated.add(objects[i].id)
                    move = self.inspect_object(positions[i])
                    move_id_tuple = (move, objects[i].id)
                    break
            if move_id_tuple is None:
                self.send_feedback('!!!! NO MORE TO EXPLORE')
                raise Exception('no more to explore')

    def get_objects_indicies_for_start(self, objects):
        try:
            white_index = self.get_index_of_type(objects, 'surmark46104')
            red_index = self.get_index_of_type(objects, ('red_totem', 'surmark950410'))
        except StopIteration:
            return None
        return white_index, red_index

    @staticmethod
    def object_classified(objects, obj_id):
        '''
        @objects list of object messages
        @obj_id id of object
        @return True of object with obj_id is classified
        '''
        for obj in objects:
            if obj.id == obj_id:
                if obj.labeled_classification == 'UNKNOWN':
                    return False
                else:
                    return True
        return False

    @txros.util.cancellableInlineCallbacks
    def prepare_to_enter(self):
        closest = []
        robot_position = (yield self.tx_pose)[0]
        def filter_and_sort(objects, positions):
            distances = np.linalg.norm(positions - robot_position, axis=1)
            argsort = np.argsort(distances)
            return argsort

        def is_done(objects, positions):
            res = self.get_objects_indicies_for_start(objects)
            if res is None:
                return None
            white_index, red_index = res
            return objects[white_index], positions[white_index], objects[red_index], positions[red_index]

        white, white_position, red, red_position = yield self.explore_closest_until(is_done, filter_and_sort)
        self.objects_passed.add(white.id)
        self.objects_passed.add(red.id)
        gate = self.get_gate(white_position, red_position, robot_position)
        self.send_feedback('Going through start gate formed by {} and {}'.format(white.id, red.id))
        yield self.go_thru_gate(gate, AFTER=-2)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        '''
        TODO: check for new objects in background, cancel move
              somefucking how handle case where gates litteraly loop back and head the other direction
        '''
        self.objects_passed = set()
        # Wait a bit for PCDAR to get setup
        yield self.nh.sleep(3.0)
        yield self.set_vrx_classifier_enabled(SetBoolRequest(data=True))
        yield self.prepare_to_enter()
        yield self.wait_for_task_such_that(lambda task: task.state =='running')
        yield self.move.forward(7.0).go()
        while not (yield self.do_next_gate()):
            pass
        self.send_feedback('This is the last gate! Going through!')
        yield self.move.forward(10).go()
        yield self.set_vrx_classifier_enabled(SetBoolRequest(data=False))

