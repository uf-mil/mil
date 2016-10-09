#!/usr/bin/env python
from navigator_msgs.msg import PerceptionObject, BuoyArray
from visualization_msgs.msg import Marker
from txros import NodeHandle, util
from twisted.internet import defer, reactor

nh = None


class ObjectClassifier:

    def __init__(self):
        print "init OC"
        self.classified_ids = {}
        self.currently_classifying = False

    @util.cancellableInlineCallbacks
    def _init(self):
        self.nh = yield NodeHandle.from_argv("object_classifier")
        global nh
        nh = self.nh

        self.pub_obj_found = yield self.nh.advertise('/classifier/object', PerceptionObject)
        self.pub_object_searching = yield self.nh.advertise('/classifier/looking_for', Marker)

        self.sub_unclassified = yield self.nh.subscribe('/unclassified/objects', BuoyArray, self.new_objects)

        defer.returnValue(self)

    @util.cancellableInlineCallbacks
    def new_objects(self, buoy_array):
        if self.currently_classifying:
            return
        self.currently_classifying = True
        buoys = buoy_array.buoys
        unclassified_buoy = None

        if(len(self.classified_ids) < 5):
            for b in buoys:
                if b.id not in self.classified_ids.keys():
                    unclassified_buoy = b
                    marker = Marker()
                    marker.header.stamp = nh.get_time()
                    marker.header.seq = 1
                    marker.header.frame_id = "enu"
                    marker.id = 0
                    marker.pose.position = unclassified_buoy.position
                    marker.type = marker.SPHERE
                    marker.action = marker.ADD
                    marker.scale.x = 1.0
                    marker.scale.y = 1.0
                    marker.scale.z = 1.0
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0

                    self.pub_object_searching.publish(marker)
                    yield nh.sleep(2)
                    x = yield util.nonblocking_raw_input("What object is this? ")

                    if(x == 'skip'):
                        self.currently_classifying = False
                        return
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
                    return

        for b in buoys:
            if b.id in self.classified_ids.keys():
                obj = PerceptionObject()
                obj.name = self.classified_ids[b.id]
                obj.position = b.position
                obj.size.x = b.height
                obj.size.y = b.width
                obj.size.z = b.depth
                obj.id = b.id
                self.pub_obj_found.publish(obj)

        self.currently_classifying = False


@util.cancellableInlineCallbacks
def main():

    od = yield ObjectClassifier()._init()

reactor.callWhenRunning(main)
reactor.run()
