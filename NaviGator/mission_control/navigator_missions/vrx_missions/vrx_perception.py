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
    def announce_object(self, obj_id, classification, position_enu):
        if classification == 'UNKNOWN': 
            print 'Ignoing UNKNOWN object {}'.format(obj_id)
            defer.returnValue(False)
        if obj_id in self.announced:
            defer.returnValue(False)
        geo_point = yield self.enu_position_to_geo_point(position_enu)
        msg = GeoPoseStamped()
        msg.header.frame_id = classification
        msg.pose.position = geo_point
        self.perception_landmark.publish(msg)
        defer.returnValue(True)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.wait_for_task_such_that(lambda task: task.state in ['running'])
        yield self.set_vrx_classifier_enabled(SetBoolRequest(data=True))
        objects = {}
        # Making assumptions about vrx. That the first object appears at 5 seconds,
        # each object lasts for 5 secodns, and a new one appears 10 seconds after the previous appeared
        next_reset = Duration.from_sec(5.0)
        next_start = Duration.from_sec(1E5)
        next_stop = Duration.from_sec(0)
        while True:
            task_info = yield self.task_info_sub.get_next_message()
            elapsed_time = task_info.elapsed_time
            if elapsed_time > next_reset:
                self.send_feedback('Resetting PCODAR')
                next_start = next_reset + Duration.from_sec(1.0)
                next_stop = next_reset + Duration.from_sec(4.5)
                next_reset += Duration.from_sec(10.0)
                self.announced = set()
                yield self.reset_pcodar()
            if elapsed_time < next_start or elapsed_time > next_stop:
                yield self.nh.sleep(0.1)
                continue
            new_objects, positions = yield self.get_object_map()
            for key in new_objects:
                if key not in objects:
                    print 'NEW object {} {}'.format(key, new_objects[key])
                    yield self.announce_object(key, new_objects[key], positions[key])
                elif objects[key] != new_objects[key]:
                    print '{} changed from {} to {}'.format(key, objects[key], new_objects[key])
                    yield self.announce_object(key, new_objects[key], positions[key])
            objects = new_objects
