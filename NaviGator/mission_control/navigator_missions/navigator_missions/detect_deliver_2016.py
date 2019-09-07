#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
import tf.transformations as trns
from navigator_msgs.msg import ShooterDoAction
from navigator_msgs.srv import CameraToLidarTransform, CameraToLidarTransformRequest
from geometry_msgs.msg import Point, PoseStamped
from twisted.internet import defer
import mil_tools
from mil_misc_tools.text_effects import fprint
from navigator_tools import MissingPerceptionObject
import genpy
from navigator import Navigator


class DetectDeliver(Navigator):
    shoot_distance_meters = 2.7
    theta_offset = np.pi / 2.0
    spotings_req = 1
    circle_radius = 10
    circle_direction = "cw"
    platform_radius = 0.925
    search_timeout_seconds = 300
    SHAPE_CENTER_TO_BIG_TARGET = 0.35
    SHAPE_CENTER_TO_SMALL_TARGET = -0.42
    WAIT_BETWEEN_SHOTS = 5  # Seconds to wait between shooting
    NUM_BALLS = 4
    LOOK_AT_TIME = 5
    FOREST_SLEEP = 15
    BACKUP_DISTANCE = 5

    def __init__(self):
        super(DetectDeliver, self).__init__()
        self.identified_shapes = {}
        self.last_shape_error = ""
        self.last_lidar_error = ""
        self.shape_pose = None
        self.align_forest_pause = False

    @classmethod
    def init(cls):
        cls.shooter_pose_sub = cls.nh.subscribe("/shooter_pose", PoseStamped)
        cls.cameraLidarTransformer = cls.nh.get_service_client(
            "/camera_to_lidar/right_right_cam", CameraToLidarTransform)
        cls.shooterLoad = txros.action.ActionClient(
            cls.nh, '/shooter/load', ShooterDoAction)
        cls.shooterFire = txros.action.ActionClient(
            cls.nh, '/shooter/fire', ShooterDoAction)
        cls.shooter_baselink_tf = yield cls.tf_listener.get_transform('/base_link', '/shooter')

    def _bounding_rect(self, points):
        np_points = map(mil_tools.rosmsg_to_numpy, points)
        xy_max = np.max(np_points, axis=0)
        xy_min = np.min(np_points, axis=0)
        return np.append(xy_max, xy_min)

    @txros.util.cancellableInlineCallbacks
    def set_shape_and_color(self):
        target = yield self.mission_params["detect_deliver_target"].get()
        if target == "BIG":
            self.target_offset_meters = self.SHAPE_CENTER_TO_BIG_TARGET
        elif target == "SMALL":
            self.target_offset_meters = self.SHAPE_CENTER_TO_SMALL_TARGET
        self.Shape = yield self.mission_params["detect_deliver_shape"].get()
        self.Color = yield self.mission_params["detect_deliver_color"].get()
        fprint("Color={} Shape={} Target={}".format(self.Color, self.Shape, target),
               title="DETECT DELIVER", msg_color='green')

    @txros.util.cancellableInlineCallbacks
    def get_waypoint(self):
        res = yield self.database_query("shooter")
        if not res.found:
            fprint("shooter waypoint not found", title="DETECT DELIVER", msg_color='red')
            raise MissingPerceptionObject("shooter", "Detect Deliver Waypoint not found")
        self.waypoint_res = res

    @txros.util.cancellableInlineCallbacks
    def get_any_shape(self):
        shapes = yield self.get_shape()
        if shapes.found:
            for shape in shapes.shapes.list:
                normal_res = yield self.get_normal(shape)
                if normal_res.success:
                    enu_cam_tf = yield self.tf_listener.get_transform(
                        '/enu',
                        '/' + shape.header.frame_id,
                        shape.header.stamp)
                    self.update_shape(shape, normal_res, enu_cam_tf)
                    defer.returnValue(((shape.Shape, shape.Color), self.identified_shapes[(shape.Shape, shape.Color)]))
                else:
                    fprint("Normal not found Error={}".format(normal_res.error),
                           title="DETECT DELIVER", msg_color='red')
        else:
            fprint("shape not found Error={}".format(shapes.error), title="DETECT DELIVER", msg_color="red")
        defer.returnValue(False)

    @txros.util.cancellableInlineCallbacks
    def circle_search(self):
        platform_np = mil_tools.rosmsg_to_numpy(self.waypoint_res.objects[0].position)
        yield self.move.look_at(platform_np).set_position(platform_np).backward(self.circle_radius)\
                       .yaw_left(90, unit='deg').go(move_type="drive")

        done_circle = False

        @txros.util.cancellableInlineCallbacks
        def do_circle():
            yield self.move.circle_point(platform_np, direction=self.circle_direction).go()
            done_circle = True  # noqa flake8 cant see that it is defined above

        do_circle()
        while not done_circle:
            res = yield self.get_any_shape()
            if res is False:
                yield self.nh.sleep(0.25)
                continue
            fprint("Shape ({}found, using normal to look at other 3 shapes if needed".format(
                res[0]), title="DETECT DELIVER", msg_color="green")
            #  circle_defer.cancel()
            shape_color, found_shape_pose = res
            if self.correct_shape(shape_color):
                self.shape_pose = found_shape_pose
                return
            # Pick other 3 to look at
            rot_right = np.array([[0, -1], [1, 0]])
            (shape_point, shape_normal) = found_shape_pose
            rotated_norm = np.append(rot_right.dot(shape_normal[:2]), 0)
            center_point = shape_point - shape_normal * (self.platform_radius / 2.0)

            point_opposite_side = center_point - shape_normal * self.circle_radius
            move_opposite_side = self.move.set_position(
                point_opposite_side).look_at(center_point).yaw_left(90, unit='deg')

            left_or_whatever_point = center_point + rotated_norm * self.circle_radius
            move_left_or_whatever = self.move.set_position(
                left_or_whatever_point).look_at(center_point).yaw_left(90, unit='deg')

            right_or_whatever_point = center_point - rotated_norm * self.circle_radius
            move_right_or_whatever = self.move.set_position(
                right_or_whatever_point).look_at(center_point).yaw_left(90, unit='deg')

            yield self.search_sides((move_right_or_whatever, move_opposite_side, move_left_or_whatever))
            return
        fprint("No shape found after complete circle", title="DETECT DELIVER", msg_color='red')
        raise Exception("No shape found on platform")

    def update_shape(self, shape_res, normal_res, tf):
        self.identified_shapes[(shape_res.Shape, shape_res.Color)] = self.get_shape_pos(normal_res, tf)

    def correct_shape(self, (shape, color)):
        return (self.Color == "ANY" or self.Color == color) and (self.Shape == "ANY" or self.Shape == shape)

    @txros.util.cancellableInlineCallbacks
    def search_side(self):
        fprint("Searching side", title="DETECT DELIVER", msg_color='green')
        start_time = self.nh.get_time()
        while self.nh.get_time() - start_time < genpy.Duration(self.LOOK_AT_TIME):
            res = yield self.get_any_shape()
            if res is not False:
                defer.returnValue(res)
            yield self.nh.sleep(0.1)
        defer.returnValue(False)

    @txros.util.cancellableInlineCallbacks
    def search_sides(self, moves):
        for move in moves:
            yield move.go(move_type="drive")
            res = yield self.search_side()
            if res is False:
                fprint("No shape found on side", title="DETECT DELIVER", msg_color='red')
                continue
            shape_color, found_pose = res
            if self.correct_shape(shape_color):
                self.shape_pose = found_pose
                return
            fprint("Saw (Shape={}, Color={}) on this side".format(
                shape_color[0], shape_color[1]), title="DETECT DELIVER", msg_color='green')

    @txros.util.cancellableInlineCallbacks
    def search_shape(self):
        shapes = yield self.get_shape()
        if shapes.found:
            for shape in shapes.shapes.list:
                normal_res = yield self.get_normal(shape)
                if normal_res.success:
                    enu_cam_tf = yield self.tf_listener.get_transform('/enu', '/' + shape.header.frame_id,
                                                                      shape.header.stamp)
                    if self.correct_shape(shape):
                        self.shape_pose = self.get_shape_pos(normal_res, enu_cam_tf)
                        defer.returnValue(True)
                    self.update_shape(shape, normal_res, enu_cam_tf)

                else:
                    if not self.last_lidar_error == normal_res.error:
                        fprint("Normal not found Error={}".format(normal_res.error),
                               title="DETECT DELIVER", msg_color='red')
                    self.last_lidar_error = normal_res.error
        else:
            if not self.last_shape_error == shapes.error:
                fprint("shape not found Error={}".format(shapes.error), title="DETECT DELIVER", msg_color="red")
            self.last_shape_error = shapes.error
        defer.returnValue(False)

    def select_backup_shape(self):
        for (shape, color), point_normal in self.identified_shapes.iteritems():
            self.shape_pose = point_normal
            if self.Shape == shape or self.Color == color:
                fprint("Correct shape not found, resorting to shape={} color={}".format(
                    shape, color), title="DETECT DELIVER", msg_color='yellow')
                return
        if self.shape_pose is None:
            raise Exception("None seen")
        fprint("Correct shape not found, resorting to random shape", title="DETECT DELIVER", msg_color='yellow')

    @txros.util.cancellableInlineCallbacks
    def align_to_target(self):
        if self.shape_pose is None:
            self.select_backup_shape()
        goal_point, goal_orientation = self.get_aligned_pose(self.shape_pose[0], self.shape_pose[1])
        move = self.move.set_position(goal_point).set_orientation(goal_orientation).forward(self.target_offset_meters)
        # Adjust for location of shooter
        move = move.left(-self.shooter_baselink_tf._p[1]).forward(-self.shooter_baselink_tf._p[0])
        fprint("Aligning to shoot at {}".format(move), title="DETECT DELIVER", msg_color='green')
        move_complete = yield move.go(move_type="skid", blind=True)
        defer.returnValue(move_complete)

    def get_shape(self):
        return self.vision_proxies["get_shape"].get_response(Shape="ANY", Color="ANY")

    def get_aligned_pose(self, enupoint, enunormal):
        aligned_position = enupoint + self.shoot_distance_meters * enunormal  # moves x meters away
        angle = np.arctan2(-enunormal[0], enunormal[1])
        aligned_orientation = trns.quaternion_from_euler(0, 0, angle)  # Align perpindicular
        return (aligned_position, aligned_orientation)

    def get_shape_pos(self, normal_res, enu_cam_tf):
        enunormal = enu_cam_tf.transform_vector(mil_tools.rosmsg_to_numpy(normal_res.normal))
        enupoint = enu_cam_tf.transform_point(mil_tools.rosmsg_to_numpy(normal_res.closest))
        return (enupoint, enunormal)

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
            normal_res.error = "UNREASONABLE NORMAL"
        defer.returnValue(normal_res)

    def normal_is_sane(self, vector3):
        return abs(mil_tools.rosmsg_to_numpy(vector3)[1]) < 0.4

    @txros.util.cancellableInlineCallbacks
    def shoot_all_balls(self):
        for i in range(self.NUM_BALLS):
            goal = yield self.shooterLoad.send_goal(ShooterDoAction())
            fprint("Loading Shooter {}".format(i), title="DETECT DELIVER", msg_color='green')
            yield goal.get_result()
            yield self.nh.sleep(2)
            goal = yield self.shooterFire.send_goal(ShooterDoAction())
            fprint("Firing Shooter {}".format(i), title="DETECT DELIVER", msg_color='green')
            yield goal.get_result()
            fprint("Waiting {} seconds between shots".format(
                self.WAIT_BETWEEN_SHOTS), title="DETECT DELIVER", msg_color='green')
            yield self.nh.sleep(self.WAIT_BETWEEN_SHOTS)

    @txros.util.cancellableInlineCallbacks
    def continuously_align(self):
        fprint("Starting Forest Align", title="DETECT DELIVER", msg_color='green')
        while True:
            shooter_pose = yield self.shooter_pose_sub.get_next_message()
            if self.align_forest_pause:
                yield self.nh.sleep(0.1)
                continue
            shooter_pose = shooter_pose.pose

            cen = np.array([shooter_pose.position.x, shooter_pose.position.y])
            yaw = trns.euler_from_quaternion([shooter_pose.orientation.x,
                                              shooter_pose.orientation.y,
                                              shooter_pose.orientation.z,
                                              shooter_pose.orientation.w])[2]
            q = trns.quaternion_from_euler(0, 0, yaw)
            p = np.append(cen, 0)
            # fprint("Forest Aligning to p=[{}] q=[{}]".format(p, q), title="DETECT DELIVER", msg_color='green')

            # Prepare move to follow shooter
            move = self.move.set_position(p).set_orientation(q).yaw_right(90, 'deg')

            # Adjust move for location of target
            move = move.forward(self.target_offset_meters)

            # Adjust move for location of launcher
            move = move.left(-self.shooter_baselink_tf._p[1]).forward(-self.shooter_baselink_tf._p[0])

            # Move away a fixed distance to make the shot
            move = move.left(self.shoot_distance_meters)

            yield move.go(move_type='bypass')

    @txros.util.cancellableInlineCallbacks
    def shoot_and_align_forest(self):
        move = yield self.align_to_target()
        if move.failure_reason != "":
            fprint("Error Aligning with target = {}. Ending mission :(".format(
                move.failure_reason), title="DETECT DELIVER", msg_color="red")
            return
        fprint("Aligned successs. Shooting while using forest realign", title="DETECT DELIVER", msg_color="green")
        align_defer = self.continuously_align()
        fprint("Sleeping for {} seconds to allow for alignment",
               title="DETECT DELIVER".format(self.FOREST_SLEEP), msg_color="green")
        yield self.nh.sleep(self.FOREST_SLEEP)
        for i in range(self.NUM_BALLS):
            goal = yield self.shooterLoad.send_goal(ShooterDoAction())
            fprint("Loading Shooter {}".format(i), title="DETECT DELIVER", msg_color='green')
            yield goal.get_result()
            yield self.nh.sleep(2)
            self.align_forest_pause = True
            goal = yield self.shooterFire.send_goal(ShooterDoAction())
            fprint("Firing Shooter {}".format(i), title="DETECT DELIVER", msg_color='green')
            yield goal.get_result()
            yield self.nh.sleep(1)
            self.align_forest_pause = False
            fprint("Waiting {} seconds between shots".format(
                self.WAIT_BETWEEN_SHOTS), title="DETECT DELIVER", msg_color='green')
            yield self.nh.sleep(self.WAIT_BETWEEN_SHOTS)
        align_defer.cancel()

    @txros.util.cancellableInlineCallbacks
    def backup_from_target(self):
        yield self.move.left(self.BACKUP_DISTANCE).go()

    @txros.util.cancellableInlineCallbacks
    def shoot_and_align(self):
        move = yield self.align_to_target()
        if move.failure_reason != "":
            fprint("Error Aligning with target = {}. Ending mission :(".format(
                move.failure_reason), title="DETECT DELIVER", msg_color="red")
            return
        fprint("Aligned successs. Shooting without realignment", title="DETECT DELIVER", msg_color="green")
        yield self.shoot_all_balls()

    @txros.util.cancellableInlineCallbacks
    def setup_mission(self):
        stc_color = yield self.mission_params["scan_the_code_color3"].get(raise_exception=False)
        if stc_color is False:
            color = "ANY"
        else:
            color = stc_color
        # color = "ANY"
        shape = "ANY"
        fprint("Setting search shape={} color={}".format(shape, color), title="DETECT DELIVER", msg_color='green')
        yield self.mission_params["detect_deliver_shape"].set(shape)
        yield self.mission_params["detect_deliver_color"].set(color)

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.setup_mission()
        fprint("STARTING MISSION", title="DETECT DELIVER", msg_color='green')
        yield self.vision_proxies["get_shape"].start()
        yield self.set_shape_and_color()  # Get correct goal shape/color from params
        yield self.get_waypoint()         # Get waypoint of shooter target
        yield self.circle_search()        # Go to waypoint and circle until target found
        #  yield self.shoot_and_align()      # Align to target and shoot
        yield self.shoot_and_align_forest()      # Align to target and shoot
        yield self.backup_from_target()
        yield self.vision_proxies["get_shape"].stop()
        fprint("ENDING MISSION", title="DETECT DELIVER", msg_color='green')
