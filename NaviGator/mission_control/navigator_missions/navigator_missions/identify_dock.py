#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool, SetBoolRequest
from twisted.internet import defer
import mil_tools
from mil_misc_tools.text_effects import fprint
from navigator_msgs.srv import GetDockBays, GetDockBaysRequest
from navigator_msgs.srv import CameraToLidarTransform, CameraToLidarTransformRequest
import genpy


def print_good(message):
    return fprint(message, title="IDENTIFY DOCK", msg_color='green')


def print_bad(message):
    return fprint(message, title="IDENTIFY DOCK", msg_color='yellow')


class IdentifyDockMission:
    CIRCLE_RADIUS = 16  # Radius of circle in dock circle pattern
    BAY_SEARCH_TIMEOUT = 30  # Seconds to circle dock looking for bays before giving up
    LOOK_AT_DISTANCE = 13  # Distance in front of bay to move to to look at shape
    LOOK_SHAPE_TIMEOUT = 10  # Time to look at bay with cam to see symbol
    DOCK_DISTANCE = 5   # Distance in front of bay point to dock to
    DOCK_SLEEP_TIME = 2   # Time to wait after docking before undocking aka show off time 1.016+
    WAYPOINT_NAME = "Dock"
    TIMEOUT_CIRCLE_VISION_ONLY = 40
    MARKER_DISTANCE = 13  # How far in front of line made my markers to go to look for dock
    MARKER_SEARCH_TIME = 30

    def __init__(self, navigator):
        self.navigator = navigator
        self.ogrid_activation_client = self.navigator.nh.get_service_client('/identify_dock/active', SetBool)
        self.get_bays = self.navigator.nh.get_service_client('/identify_dock/get_bays', GetDockBays)
        self.cameraLidarTransformer = self.navigator.nh.get_service_client(
            "/camera_to_lidar/front_right_cam", CameraToLidarTransform)
        self.identified_shapes = {}

    def start_ogrid(self):
        return self.ogrid_activation_client(SetBoolRequest(data=True))

    def stop_ogrid(self):
        return self.ogrid_activation_client(SetBoolRequest(data=False))

    @txros.util.cancellableInlineCallbacks
    def get_target_bays(self):
        bay_1_color = yield self.navigator.mission_params["dock_color_1"].get()
        bay_1_shape = yield self.navigator.mission_params["dock_shape_1"].get()
        bay_2_color = yield self.navigator.mission_params["dock_color_2"].get()
        bay_2_shape = yield self.navigator.mission_params["dock_shape_2"].get()
        self.bay_1 = (bay_1_shape, bay_1_color)
        self.bay_2 = (bay_2_shape, bay_2_color)
        print_good("Docking in Bay (Shape={}, Color={}) then (Shape={}, Color={})".format(
            bay_1_shape, bay_1_color, bay_2_shape, bay_2_color))

    @txros.util.cancellableInlineCallbacks
    def get_waypoint(self):
        res = yield self.navigator.database_query(self.WAYPOINT_NAME)
        self.dock_pose = mil_tools.rosmsg_to_numpy(res.objects[0].position)
        #  self.dock_pose = np.array([88.6192321777,-515.189880371,0])

    def call_get_bays(self):
        return self.get_bays(GetDockBaysRequest())

    @txros.util.cancellableInlineCallbacks
    def circle_dock_bays(self):
        '''
        Circle the dock until the bays are identified by the perception service
        '''
        done_circle = False

        @txros.util.cancellableInlineCallbacks
        def circle_bay():
            yield self.navigator.move.look_at(self.dock_pose).set_position(self.dock_pose)\
                                     .backward(self.CIRCLE_RADIUS).look_at(self.dock_pose).go()
            yield self.navigator.move.circle_point(self.dock_pose, direction='ccw').go()
            done_circle = True  # noqa flake8 doesn't see that it is defined above

        print_good("Circling dock to get bays")
        circle_bay()
        start_time = self.navigator.nh.get_time()
        while self.navigator.nh.get_time() - start_time < genpy.Duration(self.BAY_SEARCH_TIMEOUT) and not done_circle:
            res = yield self.call_get_bays()
            if not res.success:
                yield self.navigator.nh.sleep(0.1)
                continue
            self.bay_poses = (mil_tools.rosmsg_to_numpy(res.bays[0]),
                              mil_tools.rosmsg_to_numpy(res.bays[1]),
                              mil_tools.rosmsg_to_numpy(res.bays[2]))
            self.bay_normal = mil_tools.rosmsg_to_numpy(res.normal)
            print_good("Got GetDockShapes response")
            return
        raise Exception("Bays not found after circling or timed out")

    def correct_shape(self, (desired_shape, desired_color), (test_shape, test_color)):
        return (desired_color == "ANY" or desired_color == test_color) and\
               (desired_shape == "ANY" or desired_shape == test_shape)

    @txros.util.cancellableInlineCallbacks
    def check_bays(self):
        '''
        Go in front of each bay and look at the symbols present
        '''
        for pose in self.bay_poses:
            move = self.navigator.move.set_position(pose + self.bay_normal * self.LOOK_AT_DISTANCE).look_at(pose)
            print_good("Moving in front of bay for observation")
            yield move.go()
            start_time = self.navigator.nh.get_time()
            while self.navigator.nh.get_time() - start_time < genpy.Duration(self.LOOK_SHAPE_TIMEOUT):
                if (yield self.search_shape_vision_only()):
                    return
                yield self.navigator.nh.sleep(0.1)
            print_bad("Did not see shapes after going in front of bay")

    def update_shape(self, shape_res, normal_res, tf):
        print_good("Found (Shape={}, Color={} in a bay".format(shape_res.Shape, shape_res.Color))
        self.identified_shapes[(shape_res.Shape, shape_res.Color)] = self.get_shape_pos(normal_res, tf)

    def done_searching(self):
        print_good("CURRENT IDENTIFIED BAYS: {}".format(self.identified_shapes))
        for shape_color, point_normal in self.identified_shapes.iteritems():
            if self.correct_shape(self.bay_1, shape_color):
                for shape_color2, point_normal in self.identified_shapes.iteritems():
                    if self.correct_shape(self.bay_2, shape_color2):
                        return True
        return False

    @txros.util.cancellableInlineCallbacks
    def search_shape_vision_only(self):
        shapes = yield self.navigator.vision_proxies["get_shape_front"].get_response(Shape="ANY", Color="ANY")
        if shapes.found:
            for shape in shapes.shapes.list:
                normal_res = yield self.get_normal(shape)
                if normal_res.success:
                    enu_cam_tf = yield self.navigator.tf_listener.get_transform(
                        '/enu',
                        '/' + shape.header.frame_id,
                        shape.header.stamp)
                    self.update_shape(shape, normal_res, enu_cam_tf)
                    if (self.done_searching()):
                        defer.returnValue(True)
                else:
                    print_bad("NORMAL ERROR: {}".format(normal_res.error))
        else:
            print_bad("SHAPES ERROR: {}".format(shapes.error))
        defer.returnValue(False)

    def _bounding_rect(self, points):
        np_points = map(mil_tools.rosmsg_to_numpy, points)
        xy_max = np.max(np_points, axis=0)
        xy_min = np.min(np_points, axis=0)
        return np.append(xy_max, xy_min)

    @txros.util.cancellableInlineCallbacks
    def circle_dock_vision_only(self):
        done_circle = False

        @txros.util.cancellableInlineCallbacks
        def circle_bay():
            yield self.navigator.move.set_position(self.dock_pose).backward(self.CIRCLE_RADIUS)\
                                .look_at(self.dock_pose).go()
            yield self.navigator.move.circle_point(self.dock_pose, direction='ccw').go()
            done_circle = True  # noqa flake8 cant see that it is defined above

        print_good("Circling dock to get shapes identified")
        circle_bay()
        start_time = self.navigator.nh.get_time()

        max_dt = genpy.Duration(self.TIMEOUT_CIRCLE_VISION_ONLY)
        while self.navigator.nh.get_time() - start_time < max_dt and not done_circle:
            if (yield self.search_shape_vision_only()):
                return
            yield self.navigator.nh.sleep(0.1)
        raise Exception("Shapes not identified after circling or timed out")

    def normal_is_sane(self, vector3):
        return abs(mil_tools.rosmsg_to_numpy(vector3)[1]) < 0.4

    @txros.util.cancellableInlineCallbacks
    def get_normal(self, shape):
        req = CameraToLidarTransformRequest()
        req.header = shape.header
        req.point = Point()
        req.point.x = shape.CenterX
        req.point.y = shape.CenterY
        rect = self._bounding_rect(shape.points)
        req.tolerance = int(min(rect[0] - rect[3], rect[1] - rect[4]) / 2.0)
        normal_res = yield self.cameraLidarTransformer(req)
        if not self.normal_is_sane(normal_res.normal):
            normal_res.success = False
            print_bad("UNREASONABLE NORMAL={}".format(normal_res.normal))
            normal_res.error = "UNREASONABLE NORMAL"
        defer.returnValue(normal_res)

    def get_shape_pos(self, normal_res, enu_cam_tf):
        enunormal = enu_cam_tf.transform_vector(mil_tools.rosmsg_to_numpy(normal_res.normal))
        enupoint = enu_cam_tf.transform_point(mil_tools.rosmsg_to_numpy(normal_res.closest))
        return (enupoint, enunormal)

    @txros.util.cancellableInlineCallbacks
    def dock_in_found_bays(self):
        bay_1_shape = None
        bay_2_shape = None
        incorrect_shapes = []
        dock_func = self.dock_in_bay_blind
        for shape_color, point_normal in self.identified_shapes.iteritems():
            if self.correct_shape(self.bay_1, shape_color):
                bay_1_shape = point_normal
            elif self.correct_shape(self.bay_2, shape_color):
                bay_2_shape = point_normal
            else:
                incorrect_shapes.append(point_normal)
        if bay_1_shape is None:
            if len(incorrect_shapes) >= 1:
                print_bad("First bay not found, going in random bay")
                yield dock_func(incorrect_shapes[0])
                del incorrect_shapes[0]
            else:
                print_bad("First bay not found and no other bays found, moving on to second")
        else:
            print_good("Docking in first desired bay")
            yield dock_func(bay_1_shape)

        if bay_2_shape is None:
            if len(incorrect_shapes) >= 1:
                print_bad("Second bay not found, going in random bay")
                yield dock_func(incorrect_shapes[0])
                del incorrect_shapes[0]
            else:
                print_bad("Second bay not found and no other bays found")
        else:
            print_good("Docking in second desired bay")
            yield dock_func(bay_2_shape)

    @txros.util.cancellableInlineCallbacks
    def dock_in_bay_blind(self, (point, normal)):
        move_front = self.navigator.move.set_position(point + normal * self.LOOK_AT_DISTANCE).look_at(point)
        move_in = self.navigator.move.set_position(point + normal * self.DOCK_DISTANCE).look_at(point)
        yield move_front.go(move_type='skid')
        yield move_in.go(move_type='skid', speed_factor=[0.5, 0.5, 0.5], blind=True)
        yield self.navigator.nh.sleep(self.DOCK_SLEEP_TIME)
        yield move_front.go(move_type='skid', speed_factor=[0.5, 0.5, 0.5], blind=True)

    @txros.util.cancellableInlineCallbacks
    def dock_in_bay_using_ogrid(self, (point, normal)):
        move_front = self.navigator.move.set_position(point + normal * self.LOOK_AT_DISTANCE).look_at(point)
        move_in = self.navigator.move.set_position(point + normal * self.DOCK_DISTANCE).look_at(point)
        yield move_front.go(move_type='skid')
        yield self.start_ogrid()
        yield move_in.go(move_type='skid', speed_factor=[0.5, 0.5, 0.5])
        yield self.navigator.nh.sleep(self.DOCK_SLEEP_TIME)
        yield move_front.go(move_type='skid', speed_factor=[0.5, 0.5, 0.5])
        yield self.stop_ogrid()

    @txros.util.cancellableInlineCallbacks
    def get_dock_search_markers(self):
        left_res = yield self.navigator.database_query("DockEdgeLeft")
        left_pose = mil_tools.rosmsg_to_numpy(left_res.objects[0].position)

        right_res = yield self.navigator.database_query("DockEdgeRight")
        right_pose = mil_tools.rosmsg_to_numpy(right_res.objects[0].position)

        search_line_vector = right_pose - left_pose
        search_line_vector_normal = search_line_vector / np.linalg.norm(search_line_vector)
        if np.isnan(search_line_vector_normal[0]):
            raise Exception("Gate Edge Markers are not in a line. Perhaps they were not placed")
        rot_right = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 0]])
        search_line_rot = rot_right.dot(search_line_vector_normal)

        left_search = left_pose + search_line_rot * self.MARKER_DISTANCE
        right_search = right_pose + search_line_rot * self.MARKER_DISTANCE

        move_left = self.navigator.move.set_position(left_search).look_at(left_pose)
        move_right = self.navigator.move.set_position(right_search).look_at(right_pose)

        pose = self.navigator.pose[0][:2]
        distance_test = np.array([np.linalg.norm(pose - move_left.position[:2]),
                                  np.linalg.norm(pose - move_right.position[:2])])
        if np.argmin(distance_test) == 0:
            self.search_moves = (move_left, move_right)
        else:
            self.search_moves = (move_right, move_left)

    @txros.util.cancellableInlineCallbacks
    def search_markers(self):
        yield self.get_dock_search_markers()
        done_moves = False
        yield self.search_moves[0].go(move_type="skid")
        second_move = self.search_moves[1].go(move_type="skid")
        start_time = self.navigator.nh.get_time()
        while self.navigator.nh.get_time() - start_time < genpy.Duration(self.MARKER_SEARCH_TIME) and not done_moves:
            if (yield self.search_shape_vision_only()):
                second_move.cancel()
                return
            yield self.navigator.nh.sleep(0.1)
        raise Exception("Bays not found after searching markers or timed out")

    @txros.util.cancellableInlineCallbacks
    def find_and_dock(self):
        yield self.navigator.vision_proxies["get_shape_front"].start()
        yield self.get_target_bays()
        yield self.get_waypoint()

        # If using the Daniel/Anthony service
        #  yield self.circle_dock_bays()
        #  yield self.check_bays()

        # Circle search option
        #  yield self.circle_dock_vision_only()

        # Marker option
        yield self.search_markers()

        yield self.dock_in_found_bays()
        yield self.navigator.vision_proxies["get_shape_front"].stop()


@txros.util.cancellableInlineCallbacks
def setup_mission(navigator):
    stc_1 = yield navigator.mission_params["scan_the_code_color1"].get(raise_exception=False)
    if stc_1 is False:
        bay_1_color = "ANY"
    else:
        bay_1_color = stc_1

    stc_2 = yield navigator.mission_params["scan_the_code_color2"].get(raise_exception=False)
    if stc_2 is False:
        bay_2_color = "ANY"
    else:
        bay_2_color = stc_2

    bay_1_shape = "ANY"
    bay_2_shape = "ANY"
    yield navigator.mission_params["dock_shape_2"].set(bay_2_shape)
    yield navigator.mission_params["dock_shape_1"].set(bay_1_shape)
    yield navigator.mission_params["dock_color_1"].set(bay_1_color)
    yield navigator.mission_params["dock_color_2"].set(bay_2_color)


@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    yield setup_mission(navigator)
    print_good("STARTING MISSION")
    mission = IdentifyDockMission(navigator)
    yield mission.find_and_dock()
    print_bad("ENDING MISSION")
