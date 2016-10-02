#!/usr/bin/env python
from navigator_msgs.msg import PerceptionObject, BuoyArray
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from rawgps_common.gps import ecef_from_latlongheight, enu_from_ecef
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Vector3, Pose
from navigator_tools import rosmsg_to_numpy, odometry_to_numpy
from txros import NodeHandle, util
from twisted.internet import defer, reactor

nh = None

class ObjectClassifier:

    def __init__(self):
        print "init OC"
        self.classified_ids = []
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

    def get_input(self):
        pass


    @util.cancellableInlineCallbacks
    def new_objects(self, buoyArray):
        if self.currently_classifying:
            return
        self.currently_classifying = True
        buoys = buoyArray.buoys
        unclassified_buoy = None
        for b in buoys:
            if b.id not in self.classified_ids:
                unclassified_buoy = b
                break

        if unclassified_buoy is None:
            self.currently_classifying = False
            return

        marker = Marker()
        marker.header.stamp = nh.get_time()
        marker.header.seq = 1;
        marker.header.frame_id = "enu";     
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

        y = yield self.pub_object_searching.publish(marker)

        x = yield util.nonblocking_raw_input("What object is this? ")
        if(x == 'skip'):
            self.currently_classifying = False
            return


        obj = PerceptionObject()
        obj.name = x
        obj.position = unclassified_buoy.position
        obj.size.x = unclassified_buoy.height
        obj.size.y = unclassified_buoy.width
        obj.size.z = unclassified_buoy.depth
        obj.id = unclassified_buoy.id

        self.classified_ids.append(obj.id)

        self.pub_obj_found.publish(obj)
        self.currently_classifying = False



@util.cancellableInlineCallbacks
def main():

    od = yield ObjectClassifier()._init() 

reactor.callWhenRunning(main)
reactor.run()