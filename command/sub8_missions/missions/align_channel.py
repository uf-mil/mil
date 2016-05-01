import txros
from txros import util
import rospy
import tf

from twisted.internet import defer
from sub8_msgs.srv import SearchPose, SearchPoseRequest
from sub8_ros_tools import normalize, clip_norm, pose_to_numpy
from image_geometry import PinholeCameraModel

import numpy as np


@util.cancellableInlineCallbacks
def run(sub_singleton):
    global marker_found

    print "Searching"
    nh = yield txros.NodeHandle.from_argv('marker_alignment')
    next_pose = yield nh.get_service_client('/next_search_pose', SearchPose)
    cam = PinholeCameraModel()

    # Go to max height to see as much as we can.
    yield sub_singleton.move.depth(0.6).zero_roll_and_pitch().go()
    yield nh.sleep(1.0)

    # This is just to get camera info
    resp = yield sub_singleton.channel_marker.get_2d()
    cam.fromCameraInfo(resp.camera_info)

    intial_pose = sub_singleton.last_pose()
    intial_height = yield sub_singleton.get_dvl_range()
    raidus = calculate_visual_radius(cam, intial_height)

    s = SearchPoseRequest()
    s.intial_position = (yield intial_pose).pose.pose
    s.search_radius = raidus
    s.reset_search = True
    yield next_pose(s)
    s.reset_search = False

    marker_found = False
    goer = go_to_marker(nh, sub_singleton, cam)

    while not marker_found:
        # Move in search pattern until we find the marker
        resp = yield next_pose(s)
        print resp

        if resp.area > .8:
            continue

        print pose_to_numpy(resp.target_pose)[0]
        yield sub_singleton.move.set_position(pose_to_numpy(resp.target_pose)[0]).zero_roll_and_pitch().go()
        print "Searcher arrived"

    yield goer
    nh.shutdown()


@util.cancellableInlineCallbacks
def go_to_marker(nh, sub_singleton, cam):
    '''
    Continuously look for a marker. If one is found stop looping everywhere and go to it.
    '''
    global marker_found
    while not marker_found:
        response = yield sub_singleton.channel_marker.get_2d()
        print response.found
        marker_found = response.found
        yield nh.sleep(.5)

    # How many times should we attempt to reposition ourselves
    iterations = 4
    # Used to stop
    last_target_position = np.array([np.inf, np.inf, np.inf])
    for i in range(iterations):
        print "Iteration {}.".format(i + 1)
        response = yield sub_singleton.channel_marker.get_2d()

        if not response.found:
            # We had a problem before when we though we saw something.
            marker_found = False
            break

        target_position = yield transform_px_to_m(response, cam, sub_singleton._tf_listener)
        print target_position

        yield sub_singleton.move.set_position(target_position).yaw_left(response.pose.theta).zero_roll_and_pitch().go()

        # Check if the past two targets were found in the same place - if so, stop.
        if np.isclose(target_position[:2], last_target_position[:2], atol=.05).all():
            print "Close enough!"
            break

        last_target_position = target_position
        yield nh.sleep(5.0)


@util.cancellableInlineCallbacks
def transform_px_to_m(response, cam, tf_listener):
    '''
    Finds the position of the marker in meters relative to the current position.
    Same process as the occupancy grid maker.
    '''

    timestamp = response.header.stamp

    # Calculate position of marker accounting for camera rotation in the camera frame.
    transform = yield tf_listener.get_transform("/ground", "/downward", timestamp)
    height = transform._p[2]

    dir_vector = unit_vector(np.array([cam.cx(), cam.cy()]) - [response.pose.x, response.pose.y])

    transform = yield tf_listener.get_transform("/map", "/downward", timestamp)
    cam_rotation = tf.transformations.euler_from_quaternion(transform._q)[2] + np.pi / 2
    dir_vector = np.dot(dir_vector, make_2D_rotation(cam_rotation))

    # Calculate distance from middle of frame to marker in meters.
    magnitude = calculate_visual_radius(cam, height, second_point=[response.pose.x, response.pose.y])
    abs_position = np.append(transform._p[:2] + dir_vector[::-1] * magnitude, transform._p[2])
    defer.returnValue(abs_position)


def calculate_visual_radius(cam, height, second_point=None):
    '''
    Draws rays to find the radius of the FOV of the camera in meters.
    It also can work to find the distance between two planar points some distance from the camera.
    '''

    mid_ray = np.array([0, 0, 1])

    if second_point is None:
        if cam.cy() < cam.cx():
            second_point = np.array([cam.cx(), 0])
        else:
            second_point = np.array([0, cam.cy()])

    edge_ray = unit_vector(cam.projectPixelTo3dRay(second_point))

    # Calculate angle between vectors and use that to find r
    theta = np.arccos(np.dot(mid_ray, edge_ray))
    return np.tan(theta) * height


def unit_vector(vect):
    return vect / np.linalg.norm(vect)


def make_2D_rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s],
                     [s, c]], dtype=np.float32)
