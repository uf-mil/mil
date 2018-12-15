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
from mil_msgs.srv import CameraToLidarTransform, CameraToLidarTransformRequest
from mil_msgs.msg import ObjectsInImage
import txros
from geometry_msgs.msg import Point


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
        parser.add_argument('-t', '--time', type=int, default=-1)
        cls.parser = parser

        cls.bboxsub = cls.nh.subscribe("/bbox_pub", ObjectsInImage)

        cls.camera_lidar_tf = cls.nh.get_service_client('/camera_to_lidar/front_right_cam', CameraToLidarTransform)

    @util.cancellableInlineCallbacks
    def run(self, args):
        # Parse Arguments
        wait_time = args.time

        dock = yield self.get_sorted_objects(name='dock', n=1)
        dock = self.dock[0][0]
        dock_position = rosmsg_to_numpy(self.dock.pose.position)

        center_frame = yield self.get_center_frame()
        pose_offset = yield self.get_target_pt(center_frame)
        symbol = pose_offset[1].
        print pose_offset
        yield self.move.set_position(pose_offset).look_at(dock_position).go(blind=True)

        timeout = None
        dock_timing = False
        yield self.nh.sleep(wait_time)

        yield self.move.backward(10).go()


        self.send_feedback('Done with docking!')

    @util.cancellableInlineCallbacks
    def get_center_frame(self):
        msgf = yield self.bboxsub.get_next_message()
        msg = msgf.objects[0]
        #print msg
        c1 = rosmsg_to_numpy(msg.points[0])
        c2 = rosmsg_to_numpy(msg.points[1])
        c1 = np.array([c1[1], c1[0], 0])
        c2 = np.array([c2[1], c2[0], 0])
        c1 *= 4
        c2 *= 4
        tmp = (((c1 + c2) / 2.0), msgf, msg.find)
        #print tmp
        defer.returnValue(tmp)

    @util.cancellableInlineCallbacks
    def get_target_pt(self, center_frame):
        msg = CameraToLidarTransformRequest()
        msg.header.stamp = center_frame[1].header.stamp
        msg.header.frame_id = center_frame[1].header.frame_id
        msg.point = Point(x=center_frame[0][0], y=center_frame[0][1], z=0.0)
        msg.tolerance = 100

        #print 'msg'
        #print msg
        #print '----'

        pose_offset = yield self.camera_lidar_tf(msg)

        #print pose_offset

        #pose_offset = rosmsg_to_numpy(pose_offset.closest)


        cam_to_enu = yield self.tf_listener.get_transform('enu', center_frame[1].header.frame_id)
        normal = rosmsg_to_numpy(pose_offset.normal)
        normal = cam_to_enu.transform_vector(normal)
        normal = normal[0:2] / np.linalg.norm(normal[0:2])
        normal = np.append(normal, [0])
        found_pt = rosmsg_to_numpy(pose_offset.closest)
        found_pt = cam_to_enu.transform_point(found_pt)
        found_pt[2] = 0

        #print normal
        #print found_pt

        normal *= 3
        found_pt += normal




        #boat_pose = yield self.tx_pose
        #boat_pose = boat_pose[0]


        #pose_offset = np.array([125, 0, 0]) - boat_pose


        defer.returnValue(found_pt)