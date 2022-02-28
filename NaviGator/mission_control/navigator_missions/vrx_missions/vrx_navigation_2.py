#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from mil_tools import rosmsg_to_numpy
from std_srvs.srv import SetBoolRequest
from mil_tools import quaternion_matrix
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest, ObjectDBQueryResponse

___author___ = "Kevin Allen"


class VrxNavigation2(Vrx):
    def __init__(self, *args, **kwargs):
        super(VrxNavigation2, self).__init__(*args, **kwargs)

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
    def find_two_closest_buoys(self):

        #get total list of objects from lidar
        service_request = ObjectDBQueryRequest()
        service_request.name = "all"
        objects_msg = yield self.database_response(service_request)
        objects = objects_msg.objects

        #delete any objects that have been passed
        for i in range(len(objects)):
            if objects[i].id in self.objects_passed:
                objects.pop(i)

        #create list of buoy positions that haven't been passed
        buoy_positions = np.array([rosmsg_to_numpy(obj.pose.position) for obj in objects])

        #create list of distances of each buoy that hasn't been passed relative to the boat
        robot_position = (yield self.tx_pose)[0]
        distances = np.linalg.norm(buoy_positions - robot_position, axis=1)

        #sort those distances by index
        argsort = np.argsort(distances)

        #take the first two indicies, and obtain the two first buoys
        pos1 = distances[argsort[0]]
        pos2 = distances[argsort[1]]

        self.next_buoy1 = objects[argsort[0]]
        self.next_buoy2 = objects[argsort[1]]
        self.objects_passed.add(self.next_buoy1.id)
        self.objects_passed.add(self.next_buoy2.id)



    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):

        self.objects_passed = set()

        # Wait a bit for PCDAR to get setup
        yield self.nh.sleep(3.0)
        yield self.set_vrx_classifier_enabled(SetBoolRequest(data=True))

        yield self.find_two_closest_buoys()

        gate = self.get_gate(self.next_buoy1.pose.position, self.next_buoy2.pose.position, (yield self.tx_pose)[0])
        yield self.go_thru_gate(gate)



        yield self.move.forward(10, 'm').go()

        #yield self.prepare_to_enter()
        #yield self.wait_for_task_such_that(lambda task: task.state =='running')
        #yield self.move.forward(7.0).go()
        #while not (yield self.do_next_gate()):
        #    pass
        #self.send_feedback('This is the last gate! Going through!')
        #yield self.move.forward(10).go()
        #yield self.set_vrx_classifier_enabled(SetBoolRequest(data=False))

