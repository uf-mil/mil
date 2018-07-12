from __future__ import division
import tf
import numpy as np
from twisted.internet import defer
from txros import util
from sensor_msgs.msg import RegionOfInterest
from sensor_msgs.msg import CameraInfo
from image_geometry import PinholeCameraModel
from mil_misc_tools import text_effects
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import *

# MISSION SCRIPT
REORIENT = False

fprint = text_effects.FprintFactory(
    title="PATH_MARKER", msg_color="cyan").fprint


@util.cancellableInlineCallbacks
def run(sub):
    current_pose = sub.pose
    cam_info_sub = yield sub.nh.subscribe('/camera/down/left/camera_info',
                                          CameraInfo)

    fprint('Obtaining cam info message')
    cam_info = yield cam_info_sub.get_next_message()
    model = PinholeCameraModel()
    model.fromCameraInfo(cam_info)
    start_gate = rospy.get_param('~start_gate', [0, 0, 1])
    obs_thresh = rospy.get_param('obs_thresh', 10)

    print(sub.pose.orientation)
    get_path_marker = yield sub.nh.subscribe('/path_roi', RegionOfInterest)
    get_direction = yield sub.nh.subscribe('/path_direction', String)
    while(True):
        path_marker = yield get_path_marker.get_next_message()
        get_direction = yield get_direction.get_next_message()
        if (path_marker.x_offset != 0):
            break
        if get_direction:
            dir_obs += 1
            if dir_obs >= obs_thresh:
                yield sub.move.look_at_without_pitching(-start_gate).go()
                dir_obs = 0
                REORIENT = True
                break

    if REORIENT == True:
        if get_direction.data == 'left':
            yield sub.move.yaw_left(45).go()
        elif get_direction.data == 'right':
            yield sub.move.yaw_right(45).go()
    else:
        yield sub.move.look_at_without_pitching(-start_gate).go()
        REORIENT = True

    print('Found path marker: {}'.format(path_marker))
    center_x = path_marker.x_offset + path_marker.width / 2
    center_y = path_marker.y_offset + path_marker.height / 2

    ray, pose = yield get_transform(sub, model, np.array([center_x, center_y]))
    print("pose: ", pose)
    dvl_range = yield sub.get_dvl_range()
    point = LinePlaneCollision(np.array([0, 0, 1]), np.array(
        [0, 0, dvl_range]), ray, pose)
    print('MOVING!!!')
    point[2] = current_pose.position[2]
    yield sub.move.set_position(point).go(speed=0.2)
