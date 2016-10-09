#!/usr/bin/env python
from navigator_msgs.msg import PerceptionObject, PerceptionObjects
from visualization_msgs.msg import Marker
from txros import NodeHandle, util
from nav_msgs.msg import Odometry
from twisted.internet import defer, reactor
import numpy as np
import navigator_tools as nt

nh = None


class ObjectClassifier(object):

    def __init__(self):
        print "init OC"
        self.MIN_NUM_HITS = 10
        self.classified_ids = {}
        self.ids_to_hits = {}
        self.currently_classifying = False

    @util.cancellableInlineCallbacks
    def _init(self):
        self.nh = yield NodeHandle.from_argv("object_classifier")
        global nh
        nh = self.nh
        self.position = None
        self.rot = None

        self.pub_obj_found = yield self.nh.advertise('/classifier/object', PerceptionObject)
        self.pub_object_searching = yield self.nh.advertise('/classifier/looking_for', Marker)

        self.sub_unclassified = yield self.nh.subscribe('/unclassified/objects', PerceptionObjects, self.new_objects)
        self.sub_odom = self.nh.subscribe('odom', Odometry, self.odom_cb)

        defer.returnValue(self)

    def odom_cb(self, odom):
        self.position, self.rot = nt.odometry_to_numpy(odom)[0]

    def new_objects(self, buoy_array):
        if self.currently_classifying:
            return

        if self.position is None:
            return

        self.currently_classifying = True
        buoys = buoy_array.objects

        for b in buoys:
            v = b.height * b.width * b.depth
            if b.id not in self.classified_ids.keys() or self.classified_ids[b.id] == 'unknown':
                dist = np.linalg.norm(nt.rosmsg_to_numpy(b.position) - self.position)
                v = b.height * b.width * b.depth
                h = b.height
                print b.id, v, b.height
                print "----------"

                if dist > 25:
                    x = 'unknown'

                elif v > .6 and v < 3 and h < 2:
                    x = 'buoy_' + str(b.id)

                elif v > 70 and v < 90 and h > 3.0 and h < 4.0:
                    x = 'shooter'

                elif v > 10 and v < 15 and h > 1.7 and h < 4:
                    x = 'scan_the_code'

                else:
                    continue

                if b.id not in self.ids_to_hits.keys():
                    self.ids_to_hits[b.id] = [x, 1]
                    continue

                print "NUMHITS:", self.ids_to_hits[b.id]

                if self.ids_to_hits[b.id][0] == x:
                    self.ids_to_hits[b.id][1] += 1
                else:
                    self.ids_to_hits[b.id][0] = x
                    self.ids_to_hits[b.id][1] = 1

                if self.ids_to_hits[b.id][1] < self.MIN_NUM_HITS:
                    continue

                self.classified_ids[b.id] = x

                obj = PerceptionObject()
                obj.name = self.classified_ids[b.id]
                obj.position = b.position
                obj.size.x = b.height
                obj.size.y = b.width
                obj.size.z = b.depth
                obj.id = b.id
                self.pub_obj_found.publish(obj)

        self.currently_classifying = False

        print "*******"


@util.cancellableInlineCallbacks
def main():

    od = yield ObjectClassifier()._init()

reactor.callWhenRunning(main)
reactor.run()
