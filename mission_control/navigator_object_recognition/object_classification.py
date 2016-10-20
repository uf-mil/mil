#!/usr/bin/env python
from navigator_msgs.msg import PerceptionObject, PerceptionObjects
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from txros import NodeHandle, util
from nav_msgs.msg import Odometry
from twisted.internet import defer, reactor
import numpy as np
import navigator_tools as nt
import tf

nh = None
UP = np.array([0.0, 0.0, 1.0], np.float64)
EAST, NORTH, WEST, SOUTH = [tf.transformations.quaternion_about_axis(np.pi / 2 * i, UP) for i in xrange(4)]


class ObjectClassifier(object):

    def __init__(self):
        print "init OC"
        self.MIN_NUM_HITS = 3
        self.classified_ids = {}
        self.ids_to_hits = {}
        self.currently_classifying = False
        self.all_objects = []
        self.human_mode = True

    @util.cancellableInlineCallbacks
    def _init(self):
        self.nh = yield NodeHandle.from_argv("object_classifier")
        global nh
        nh = self.nh
        self.position = None
        self.rot = None

        self.pub_obj_found = yield self.nh.advertise('/classifier/object', PerceptionObject)
        self.pub_object_searching = yield self.nh.advertise('/classifier/looking_for', Marker)

        self.sub_clicked_point = self.nh.subscribe('/clicked_point', PointStamped, self.cp_cb)
        self.sub_unclassified = yield self.nh.subscribe('/unclassified/objects', PerceptionObjects, self.new_objects)
        self.sub_odom = yield self.nh.subscribe('odom', Odometry, self.odom_cb)

        defer.returnValue(self)

    def odom_cb(self, odom):
        self.position, self.rot = nt.odometry_to_numpy(odom)[0]

    @util.cancellableInlineCallbacks
    def cp_cb(self, point_stamped):
        print "sup"
        self.currently_classifying = True
        point = nt.rosmsg_to_numpy(point_stamped.point)
        min_obj = None
        min_dist = 10000
        for b in self.all_objects:
            obj_pose = nt.rosmsg_to_numpy(b.position)
            dist = np.linalg.norm(obj_pose - point)
            print dist
            if dist < min_dist:
                min_obj = b
                min_dist = dist

        marker = Marker()
        marker.header.stamp = nh.get_time()
        marker.header.seq = 1
        marker.header.frame_id = "enu"
        marker.id = min_obj.id
        marker.pose.position = min_obj.position
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
        yield nh.sleep(1)
        x = yield util.nonblocking_raw_input("What object is this? " + str(marker.id))

        if(x == 'skip'):
            self.currently_classifying = False
            return
        self.classified_ids[min_obj.id] = x

        obj = PerceptionObject()
        obj.name = self.classified_ids[min_obj.id]
        obj.position = min_obj.position
        obj.size.x = min_obj.height
        obj.size.y = min_obj.width
        obj.size.z = min_obj.depth
        obj.id = min_obj.id
        self.pub_obj_found.publish(obj)

        marker_del = Marker()
        marker_del.action = 3  # This is DELETEALL
        self.pub_object_searching.publish(marker_del)

        self.currently_classifying = False

    def new_objects(self, buoy_array):
        if self.human_mode and not self.currently_classifying:
            self.all_objects = buoy_array.buoys
            return

        if self.currently_classifying:
            return

        if self.position is None:
            return

        self.currently_classifying = True
        buoys = buoy_array.objects

        for b in buoys:
            v = b.height * b.width * b.depth
            if b.id not in self.classified_ids.keys() or 'un' in self.classified_ids[b.id]:
                dist = np.linalg.norm(nt.rosmsg_to_numpy(b.position) - self.position)
                v = b.height * b.width * b.depth
                h = b.height
                dir_vec = (nt.rosmsg_to_numpy(b.position) - self.position) / dist
                dir_vec[2] = 0
                yaw = tf.transformations.euler_from_quaternion(self.rot)[2]
                nav_vec = [np.cos(yaw), np.sin(yaw), 0]
                mydot = np.dot(dir_vec, nav_vec)
                angle = np.arccos(mydot)

                print "id: ", b.id, yaw, mydot, angle

                if angle > (3.14 / 6):
                    continue

                print "id _ made it: ", b.id
                print "----------"

                if dist > 25:
                    x = 'un' + str(b.id)

                elif v > 1 and v < 3 and h < 2:
                    x = 'b_' + str(b.id)

                elif v > 70 and h > 3.0 and h < 5.0:
                    x = 'shooter'

                elif v > 10 and v < 15 and h > 1.7 and h < 4:
                    x = 'scan_the_code'

                else:
                    x = 'una_' + str(b.id)

                if b.id not in self.ids_to_hits.keys():
                    self.ids_to_hits[b.id] = [x, 1]
                    continue

                # print "NUMHITS:", self.ids_to_hits[b.id]

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
