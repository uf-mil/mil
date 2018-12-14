#!/usr/bin/env python
from txros import util
from navigator_missions.navigator import Navigator
import numpy as np
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer
import math
from mil_misc_tools import ThrowingArgumentParser
import tf.transformations as tform
from detect_deliver_find import DetectDeliverFind


class DiscountDocking(Navigator):

    @classmethod
    def decode_parameters(cls, parameters):
        argv = parameters.split()
        return cls.parser.parse_args(argv)

    @classmethod
    def init(cls):
        cls.detect_deiliver_find = DetectDeliverFind()

    @util.cancellableInlineCallbacks
    def run(self, args):
        # Parse Arguments

        #yield self.detect_deiliver_find.run(self.detect_deiliver_find.decode_parameters(args))

        yield self.move.forward(5).go()

        yield self.navigator.nh.sleep(10)

        yield self.move.backward(5).go()
        yield self.move.backward(5).go()
        yield self.move.backward(5).go()


        self.send_feedback('Done!')

    @util.cancellableInlineCallbacks
    def find_dock(self, override_scale):
        # Get Dock
        self.dock = yield self.get_sorted_objects(name='dock', n=1)
        self.dock = self.dock[0][0]

        # Get dock parameters
        self.dock_position = rosmsg_to_numpy(self.dock.pose.position)
        self.dock_orientation = rosmsg_to_numpy(self.dock.pose.orientation)
        self.dock_scale = rosmsg_to_numpy(self.dock.scale)
        self.dock_orientation = tform.euler_from_quaternion(self.dock_orientation)

        # If scale should be overwritten
        if override_scale:
            # Write long/short appropriately
            if self.dock_scale[0] > self.dock_scale[1]:
                self.dock_scale[0] = self.DOCK_SIZE_LONG
                self.dock_scale[1] = self.DOCK_SIZE_SHORT
            else:
                self.dock_scale[0] = self.DOCK_SIZE_SHORT
                self.dock_scale[1] = self.DOCK_SIZE_LONG

    def calculate_docking_positions(self, scan_dist, boat_pose):
        angle = self.dock_orientation[2]
        self.scans = []

        for i in range(0, 4):
            # Calculate scan point and point to look at
            pt = np.array([math.cos(angle) * dock_start_dist + self.dock_position[0],
                           math.sin(angle) * dock_start_dist + self.dock_position[1],
                           self.dock_position[2]])
            lpt = pt + np.array([math.sin(angle), -math.cos(angle), 0])

            # Calculate distance from scan point to dock and boat
            ptdist = np.linalg.norm(pt - self.dock_position)
            btdist = np.linalg.norm(pt - boat_pose)

            # Store scan
            scan = (pt, lpt, ptdist, angle, btdist)
            self.scans.append(scan)

            # 4 sides to dock
            angle -= math.pi / 2