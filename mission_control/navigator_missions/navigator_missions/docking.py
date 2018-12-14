#!/usr/bin/env python
from txros import util
from navigator_missions.navigator import Navigator
import numpy as np
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer
import math
from mil_misc_tools import ThrowingArgumentParser
import tf.transformations as tform
import time
from mil_msgs.srv import CameraToLidarTransform
from mil_msgs.msg import ObjectInImage
import txros
from geometry_msgs import Point


class Docking(Navigator):

    @classmethod
    def decode_parameters(cls, parameters):
        argv = parameters.split()
        return cls.parser.parse_args(argv)

    @classmethod
    def init(cls):
        parser = ThrowingArgumentParser(description='Dock',
                                        usage='''Default parameters: \'runtask Docking
                                         \'''')
        parser.add_argument('-t', '--time', type=int, default=15)
        cls.parser = parser

        cls.bboxsub = cls.nh.subscribe("/bbox_pub", ObjectInImage)

        #cls.camera_lidar_tf = cls.nh.get_service_client('/camera_lidar_transformer', CameraToLidarTransform)

    @util.cancellableInlineCallbacks
    def run(self, args):
        # Parse Arguments
        wait_time = args.time

        #dock = yield self.get_sorted_objects(name='dock', n=1)
        #dock = self.dock[0][0]
        #dock_position = rosmsg_to_numpy(self.dock.pose.position)
        dock_position = np.array([58, -382, 0])

        timeout = None
        dock_timing = False
        while True:
            center_frame = yield self.get_center_frame()
            pose_offset = yield self.get_pose_offset(center_frame)
            target_pose_offset = np.array([5, 0, 0])
            diff_pose = pose_offset - target_pose_offset

            print pose_offset
            print("DELTA:      " + str(diff_pose))

            #yield self.move.forward(diff_pose[0]).left(diff_pose[1]).look_at(self.dock_position).go()


            if not dock_timing and pose_offset[0] < 2 and pose_offset[1] < 2:
                dock_timing = True
                timeout = time.time() + 20

            if dock_timing and time.time() > timeout:
                break

        #yield self.move.backward(10).go()


        self.send_feedback('Done with docking!')

    @util.cancellableInlineCallbacks
    def get_center_frame(self):
        msg = yield self.bboxsub.get_next_message()
        print msg
        tmp = (rosmsg_to_numpy(msg.points[0]) + rosmsg_to_numpy(msg.points[1]) / 2.0)
        print tmp
        defer.returnValue(tmp)

    @util.cancellableInlineCallbacks
    def get_pose_offset(self, center_frame):
        msg = CameraToLidarTransform()
        msg.header.stamp = self.nh.get_time()
        msg.point = Point(center_frame[0] * 720, center_frame[1] * 480)
        msg.tolerance = 20

        pose_offset = yield self.camera_lidar_tf(msg)

        print pose_offset

        pose_offset = pose_offset.closest


        #boat_pose = yield self.tx_pose
        #boat_pose = boat_pose[0]


        #pose_offset = np.array([125, 0, 0]) - boat_pose


        defer.returnValue(pose_offset)