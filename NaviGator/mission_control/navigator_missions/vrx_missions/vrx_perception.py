#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from vrx import Vrx
from mil_tools import rosmsg_to_numpy
from geographic_msgs.msg import GeoPoseStamped
from std_srvs.srv import TriggerRequest, SetBoolRequest
from genpy import Duration
from dynamic_reconfigure.msg import BoolParameter, DoubleParameter, IntParameter

___author___ = "Kevin Allen"


class VrxPerception(Vrx):
    def __init__(self, *args, **kwargs):
        self.announced = set()
        super(VrxPerception, self).__init__(*args, **kwargs)

    @txros.util.cancellableInlineCallbacks
    def get_object_map(self):
        '''
        Gets the latest objects
        returns tuple (object_dict, position_dict)
        object_dict: maps object id to classification
        position_dict: maps object_id to position in enu as numpy array
        '''
        current_objects_msg = (yield self.database_query(name='all')).objects
        ret = {}
        positions = {}
        for obj in current_objects_msg:
            classification = obj.labeled_classification
            ret[obj.id] = classification
            positions[obj.id] = rosmsg_to_numpy(obj.pose.position)
        defer.returnValue((ret, positions))

    @txros.util.cancellableInlineCallbacks
    def announce_object(self, obj_id, classification, position_enu, boat_position_enu):
        if classification == 'UNKNOWN': 
            self.send_feedback('Ignoing UNKNOWN object {}'.format(obj_id))
            defer.returnValue(False)
        if obj_id in self.announced:
            defer.returnValue(False)
        if np.linalg.norm(position_enu - boat_position_enu) < 3.5:
            self.send_feedback('Ignoring {} b/c its too close'.format(obj_id))
            defer.returnValue(False)
        geo_point = yield self.enu_position_to_geo_point(position_enu)
        msg = GeoPoseStamped()
        msg.header.frame_id = classification
        msg.pose.position = geo_point
        self.perception_landmark.publish(msg)
        defer.returnValue(True)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        p1 = BoolParameter(name='associator_forget_unseen', value=True)
        p2 = DoubleParameter(name='cluster_tolerance_m', value=0.25)
        p3 = DoubleParameter(name='associator_max_distance', value=0.25)
        p4 = IntParameter(name='cluster_min_points', value=1)
        p5 = IntParameter(name='persistant_cloud_filter_min_neighbors', value=1)
        yield self.pcodar_set_params(bools=[p1], doubles=[p2, p3], ints=[p4, p5])
        # TODO: use PCODAR amnesia to avoid this timing fiasco
        yield self.wait_for_task_such_that(lambda task: task.state in ['running'])
        yield self.set_vrx_classifier_enabled(SetBoolRequest(data=True))
        objects = {}
        while True:
            task_info = yield self.task_info_sub.get_next_message()
            new_objects, positions = yield self.get_object_map()
            position_enu = (yield self.tx_pose)[0]
            for key in new_objects:
                if key not in objects:
                    self.send_feedback('NEW object {} {}'.format(key, new_objects[key]))
                    yield self.announce_object(key, new_objects[key], positions[key], position_enu)
                elif objects[key] != new_objects[key]:
                    self.send_feedback('{} changed from {} to {}'.format(key, objects[key], new_objects[key]))
                    yield self.announce_object(key, new_objects[key], positions[key], position_enu)
            objects = new_objects
