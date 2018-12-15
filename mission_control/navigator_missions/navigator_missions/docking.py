#!/usr/bin/env python
from txros import util
from navigator_missions.navigator import Navigator
import numpy as np
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer
from mil_misc_tools import ThrowingArgumentParser
from mil_msgs.srv import CameraToLidarTransform, CameraToLidarTransformRequest
from mil_msgs.msg import ObjectsInImage
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

        # Find Dock
        dock_position = None
        largest_size = 0
        boat_pos = (yield self.tx_pose)[0]
        # Get 10 closest unclassified objects
        unclass = yield self.get_sorted_objects(name='UNKNOWN', n=10, throw=False)
        for obj in unclass[0]:
            point = rosmsg_to_numpy(obj.pose.position)
            scale = rosmsg_to_numpy(obj.scale)

            # Filter such that we know the dock is closer than 20 meters
            if np.linalg.norm(point - boat_pos) > 20:
                break

            size = scale[0] * scale[1]

            if size > largest_size:
                largest_size = size
                dock_position = point

        if dock_position is None:
            self.send_feedback('Cancelling, failed to find dock position')
            return

        self.send_feedback('Found dock, looking for image')

        # Find the camera input
        center_frame = yield self.get_center_frame()
        symbol = center_frame[2].lower()

        self.send_feedback('Identified {}'.format(symbol))

        # Find the target point
        target_pt = yield self.get_target_pt(center_frame)

        self.send_feedback('Identified target')

        # Identify the time to wait in the dock
        if wait_time == -1:
            if 'triangle' in symbol:
                wait_time = 7
            elif 'circle' in symbol:
                wait_time = 17
            else:  # Cruciform
                wait_time = 27

        # Go to pose
        self.send_feedback('Moving into dock')
        yield self.move.set_position(target_pt).look_at(dock_position).go(blind=True)

        # Sleep the appropriate amount of time
        self.send_feedback('------------------------------------------------')
        self.send_feedback('!!!!!!!!!!!!! STATION KEEPING !!!!!!!!!!!!!!!!!!')
        yield self.nh.sleep(wait_time)
        self.send_feedback('!!!!!!!!!!!!!!! EXITING DOCK !!!!!!!!!!!!!!!!!!!')
        self.send_feedback('------------------------------------------------')

        # Back out of the dock
        yield self.move.backward(5).go(blind=True)
        yield self.move.backward(5).go(blind=True)

        self.send_feedback('Done with docking!')

    @util.cancellableInlineCallbacks
    def get_center_frame(self):
        msgf = yield self.bboxsub.get_next_message()
        msg = msgf.objects[0]
        # print msg
        c1 = rosmsg_to_numpy(msg.points[0])
        c2 = rosmsg_to_numpy(msg.points[1])
        tmp = (((c1 + c2) / 2.0), msgf, msg.name)
        defer.returnValue(tmp)

    @util.cancellableInlineCallbacks
    def get_target_pt(self, center_frame):
        msg = CameraToLidarTransformRequest()
        msg.header.stamp = center_frame[1].header.stamp
        msg.header.frame_id = center_frame[1].header.frame_id
        msg.point = Point(x=center_frame[0][0], y=center_frame[0][1], z=0.0)
        msg.tolerance = 500

        pose_offset = yield self.camera_lidar_tf(msg)

        cam_to_enu = yield self.tf_listener.get_transform('enu', center_frame[1].header.frame_id)
        normal = rosmsg_to_numpy(pose_offset.normal)
        normal = cam_to_enu.transform_vector(normal)
        normal = normal[0:2] / np.linalg.norm(normal[0:2])
        normal = np.append(normal, [0])
        found_pt = rosmsg_to_numpy(pose_offset.closest)
        found_pt = cam_to_enu.transform_point(found_pt)
        found_pt[2] = 0

        # Extend out by normal multiplier
        normal *= 3
        found_pt_1 = found_pt + normal
        found_pt_2 = found_pt + -1 * normal

        # Which is closer
        boat_pos = (yield self.tx_pose)[0]
        if np.linalg.norm(found_pt_1 - boat_pos) > np.linalg.norm(found_pt_2 - boat_pos):
            found_pt = found_pt_2
        else:
            found_pt = found_pt_1

        defer.returnValue(found_pt)
