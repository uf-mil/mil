#!/usr/bin/env python
import roslib
import sys
import rospy
from twisted.internet import defer, reactor
from txros import action, util, tf, NodeHandle
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from navigator_tools import rosmsg_to_numpy, odometry_to_numpy
from rawgps_common.gps import ecef_from_latlongheight, enu_from_ecef
from navigator_msgs.srv import ScanTheCodeMission, ScanTheCodeMissionResponse
from object_classification import ObjectClassifier

class ObjectDetector:

    def __init__(self):
        print "sup"
        
    @util.cancellableInlineCallbacks
    def _init(self, bounding_box_lat_long):
        print "node_init"
        self.nh = yield NodeHandle.from_argv("object_detector")
        print "inited"
        init_bounding_box = False
        self.ecef_pose = None
        self.pose = None
        self.bounding_box = []
        self.bounding_box_lat_long = bounding_box_lat_long
        self.count = 0
        self.set_bouding_box = False
        self.object_classifier = yield ObjectClassifier()._init(self.nh)
        print "okkk"
        self.pub = yield self.nh.advertise('/rviz/gps', Marker)
        # self.odom = yield self.nh.subscribe('/odom', Odometry, self.set_odom)
        # self.absodom = yield self.nh.subscribe('/absodom', Odometry, self.set_absodom)
        self.markers = yield self.nh.subscribe('/markers_small_batcave', MarkerArray, self.add_marker)
        print "init node"

    def set_absodom(self, odom):
        self.absodom.shutdown()
        self.ecef_pose = odometry_to_numpy(odom)[0]
        print "absodom"
        if(self.pose is not None and self.ecef_pose is not None):
            pass
            #self.to_lat_long()
            
    def set_odom(self, odom):
        self.odom.shutdown()
        self.pose = odometry_to_numpy(odom)[0]
        print "odom"
        if(self.pose is not None and self.ecef_pose is not None):
            pass
            #self.to_lat_long()

    def to_lat_long(self, alt=0):
        for i, ll in enumerate(self.bounding_box_lat_long):
            lat = ll[0]
            lon = ll[1]
            ecef_pos, enu_pos = self.ecef_pose[0], self.pose[0]

            # These functions want radians
            lat, lon = np.radians([lat, lon], dtype=np.float64)
            ecef_vector = ecef_from_latlongheight(lat, lon, alt) - ecef_pos
            enu_vector = enu_from_ecef(ecef_vector, ecef_pos)
            enu_vector[2] = 0  # We don't want to move in the z at all
            pos = np.array(enu_pos + enu_vector)
            self.bounding_box.append(pos)
            self.add_marker_vis(pos, [1,1,0])

        self.set_bouding_box = True
            

    def add_marker_vis(self, pos, colors):
        marker = Marker()
        marker.header.frame_id = "enu";
        marker.header.stamp = self.nh.get_time()
        marker.ns = "my_namespace";
        marker.id = self.count
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = colors[0]
        marker.color.g = colors[1]
        marker.color.b = colors[2]
        self.pub.publish(marker);
        self.count+=1



    def add_marker(self, marker_array):
        for i, marker in enumerate(marker_array.markers):
            pos = marker.pose.position
            # print i, pos, '\n', marker.scale
            # print "---"
            self.object_classifier.classify(pos, marker.scale)
            #self.add_marker_vis([pos.x,pos.y,pos.z],[1,0,0])
            #if(self.in_bounding_box([pos.x,pos.y,pos.z])):
                #self.add_marker_vis([pos.x,pos.y,pos.z],[1,0,0])
                #self.object_classifier.classify(pos, marker.scale)
        # print "----------------"

    def in_bounding_box(self, pos):
        if(not self.set_bouding_box):
            return
        ab = np.subtract(self.bounding_box[0][:2], self.bounding_box[1][:2])
        ac = np.subtract(self.bounding_box[0][:2], self.bounding_box[2][:2])
        am = np.subtract(self.bounding_box[0][:2], pos[:2])
        if(np.dot(ab,ac) > .1):
            ac = np.subtract(self.bounding_box[0][:2], self.bounding_box[3][:2])
        
        if(0 <= np.dot(ab,am) <= np.dot(ab,ab) and 0 <= np.dot(am,ac) <= np.dot(ac,ac)):
            return True

        return False


@util.cancellableInlineCallbacks
def main():

    od = yield ObjectDetector()._init([[29.534647,-82.304280],[29.535011,-82.303323],[29.533803,-82.302639],[29.533400,-82.303577]])  


reactor.callWhenRunning(main)
reactor.run()


# util.launch_main(main)

# def launch_main(main_func):
#     @defer.inlineCallbacks
#     def _():
#         try:
#             yield main_func()
#         except Exception:
#             traceback.print_exc()
#         reactor.stop()
