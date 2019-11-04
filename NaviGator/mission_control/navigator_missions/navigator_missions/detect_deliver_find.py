#!/usr/bin/env python
from txros import util
from navigator_missions.navigator import Navigator
import numpy as np
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer
import math
from mil_misc_tools import ThrowingArgumentParser
import tf.transformations as tform
from mil_msgs.msg import ObjectsInImage


class DetectDeliverFind(Navigator):
    DOCK_SIZE_LONG = 16.0
    DOCK_SIZE_SHORT = 8.0

    CIRCLE_DISTANCE = 8.0 + math.sqrt((DOCK_SIZE_SHORT / 2)**2 + (DOCK_SIZE_LONG / 2)**2)

    @classmethod
    def decode_parameters(cls, parameters):
        argv = parameters.split()
        return cls.parser.parse_args(argv)

    @classmethod
    def init(cls):
        parser = ThrowingArgumentParser(description='Detect Deliver Find',
                                        usage='''Default parameters: \'runtask DetectDeliverFind
                                         \'''')
        parser.add_argument('-l', '--longscan', action='store_true',
                            help='set to scan the long side')
        parser.add_argument('-s', '--shortscan', action='store_true',
                            help = 'set to scan the short side')
        parser.add_argument('-o', '--overridescale', action='store_true',
                            help='''setting causes manual dock size to replace scale, where scale is only
                                    used to determine which side is longer''')
        parser.add_argument('-c', '--circle', action='store_true',
                            help='''setting causes navigator to circle the dock once first in order to help
                                    PCODAR gather enough information to produce scale and an accurate orientation''')
        parser.add_argument('-d', '--scandist', type=int, default=6,
                            help='distance to scan the images from')
        parser.add_argument('-i', '--lookin', action='store_true',
                            help='look into the dock at the end instead of being side ways')
        parser.add_argument('-e', '--enddist', type=int, default=5,
                            help='the distance to go to as the end point')
        cls.parser = parser

        cls.bboxsub = cls.nh.subscribe("/bbox_pub", ObjectsInImage)

    @util.cancellableInlineCallbacks
    def run(self, args):
        # Parse Arguments
        self.long_scan = args.longscan
        self.short_scan = args.shortscan
        override_scale = args.overridescale
        pre_circle = args.circle
        scan_dist = args.scandist
        look_in = args.lookin
        end_dist = args.enddist

        # Turn on ML classifier for docks
        yield self.set_vision_dock()

        # Find the dock
        yield self.find_dock(override_scale)

        # Get the boat pose
        boat_pose = yield self.tx_pose
        boat_pose = boat_pose[0]

        # If extra scanning circle is enabled, circle
        if pre_circle:
            start_vect = (boat_pose - self.dock_position) / np.linalg.norm(boat_pose - self.dock_position)
            start_pt = self.dock_position + start_vect * self.CIRCLE_DISTANCE
            yield self.move.set_position(start_pt).look_at(self.dock_position).yaw_right(1.57).go()
            yield self.move.circle_point(self.dock_position).go()

        # Find the dock
        yield self.find_dock(override_scale)

        # Get the boat pose
        boat_pose = yield self.tx_pose
        boat_pose = boat_pose[0]

        # Determine scan points
        self.build_scan_positions(scan_dist, boat_pose)

        # If we don't want to scan all, remove scans on long side
        if self.short_scan and not self.long_scan:
            # Remove 1 and 3; these are the long side ones.
            self.scans = [self.scans[1], self.scans[3]]
        elif self.long_scan and not self.short_scan:
            # Remove 0 and 2; these are the short side ones.
            self.scans = [self.scans[0], self.scans[2]]

        # Determine the closest scan point
        scanbtdists = np.array([sc[4] for sc in self.scans])
        closest_scan = np.argmin(scanbtdists)

        # Scan each point, stop and set correct, correct_scan, when correct found.
        correct = False
        correct_scan_idx = -1
        for i in range(closest_scan, len(self.scans)) + range(0, closest_scan):
            #yield self.move.set_position(self.scans[i][0]).look_at(self.scans[i][1]).go()
            yield self.move.set_position(self.scans[i][0]).look_at(self.dock_position).go()
            correct = yield self.scan_image()
            if correct:
                correct_scan_idx = i
                break

        # No correct scan image found. Exiting.
        if not correct:
            self.send_feedback('Image not found')
            return

        correct_scan = self.scans[correct_scan_idx]

        # Calculate closer location
        angle = correct_scan[3]
        dist = end_dist
        pt = np.array([math.cos(angle) * (dist + self.dock_scale[0] / 2) + self.dock_position[0],
                       math.sin(angle) * (dist + self.dock_scale[1] / 2) + self.dock_position[1],
                       self.dock_position[2]])
        if look_in:
            lpt = self.dock_position
        else:
            lpt = pt + np.array([math.sin(angle), -math.cos(angle), 0])

        # Go to closer location
        yield self.move.set_position(pt).look_at(lpt).go(blind=True)

        self.send_feedback('Done! In position to line up for shot.')

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

    def build_scan_positions(self, scan_dist, boat_pose):
        angle = self.dock_orientation[2]
        self.scans = []

        for i in range(0, 4):
            # Calculate scan point and point to look at
            pt = np.array([math.cos(angle) * (scan_dist + self.dock_scale[0] / 2) + self.dock_position[0],
                           math.sin(angle) * (scan_dist + self.dock_scale[1] / 2) + self.dock_position[1],
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

    @util.cancellableInlineCallbacks
    def scan_image(self):
        msgf = yield self.bboxsub.get_next_message()
        for msg in msgf.objects:
            if not (('circle' in msg.name) or ('triangle' in msg.name) or ('cruciform' in msg.name)):
                continue

            if self.long_scan:
                self.docking_scan = msg.name
                defer.returnValue(True)
            elif self.short_scan:
                if self.docking_scan is msg.name:
                    defer.returnValue(True)
        defer.returnValue(False)
