#!/usr/bin/env python3
from __future__ import annotations

import genpy
import mil_tools
import numpy as np
import tf.transformations as trns
import txros
from geometry_msgs.msg import Point, PoseStamped
from mil_misc_tools.text_effects import fprint
from navigator import Navigator
from navigator_msgs.msg import ShooterDoAction
from navigator_msgs.srv import CameraToLidarTransform, CameraToLidarTransformRequest
from navigator_tools import MissingPerceptionObject


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

    nh: txros.NodeHandle

    def __init__(self):
        super().__init__()
        self.identified_shapes = {}
        self.last_shape_error = ""
        self.last_lidar_error = ""
        self.shape_pose = None
        self.align_forest_pause = False

    @classmethod
    async def init(cls):
        cls.shooter_pose_sub = cls.nh.subscribe("/shooter_pose", PoseStamped)
        await cls.shooter_pose_sub.setup()
        cls.cameraLidarTransformer = cls.nh.get_service_client(
            "/camera_to_lidar/right_right_cam", CameraToLidarTransform
        )
        cls.shooterLoad = txros.action.ActionClient(
            cls.nh, "/shooter/load", ShooterDoAction
        )
        cls.shooterFire = txros.action.ActionClient(
            cls.nh, "/shooter/fire", ShooterDoAction
        )
        cls.shooter_baselink_tf = await cls.tf_listener.get_transform(
            "/base_link", "/shooter"
        )

    @classmethod
    async def shutdown(cls):
        await cls.shooter_pose_sub.shutdown()

    def _bounding_rect(self, points):
        np_points = map(mil_tools.rosmsg_to_numpy, points)
        xy_max = np.max(np_points, axis=0)
        xy_min = np.min(np_points, axis=0)
        return np.append(xy_max, xy_min)

    async def set_shape_and_color(self):
        target = await self.mission_params["detect_deliver_target"].get()
        if target == "BIG":
            self.target_offset_meters = self.SHAPE_CENTER_TO_BIG_TARGET
        elif target == "SMALL":
            self.target_offset_meters = self.SHAPE_CENTER_TO_SMALL_TARGET
        self.Shape = await self.mission_params["detect_deliver_shape"].get()
        self.Color = await self.mission_params["detect_deliver_color"].get()
        fprint(
            f"Color={self.Color} Shape={self.Shape} Target={target}",
            title="DETECT DELIVER",
            msg_color="green",
        )

    async def get_waypoint(self):
        res = await self.database_query("shooter")
        if not res.found:
            fprint(
                "shooter waypoint not found", title="DETECT DELIVER", msg_color="red"
            )
            raise MissingPerceptionObject(
                "shooter", "Detect Deliver Waypoint not found"
            )
        self.waypoint_res = res

    async def get_any_shape(self):
        shapes = await self.get_shape()
        if shapes.found:
            for shape in shapes.shapes.list:
                normal_res = await self.get_normal(shape)
                if normal_res.success:
                    enu_cam_tf = await self.tf_listener.get_transform(
                        "/enu", "/" + shape.header.frame_id, shape.header.stamp
                    )
                    self.update_shape(shape, normal_res, enu_cam_tf)
                    return (
                        (shape.Shape, shape.Color),
                        self.identified_shapes[(shape.Shape, shape.Color)],
                    )
                else:
                    fprint(
                        f"Normal not found Error={normal_res.error}",
                        title="DETECT DELIVER",
                        msg_color="red",
                    )
        else:
            fprint(
                f"shape not found Error={shapes.error}",
                title="DETECT DELIVER",
                msg_color="red",
            )
        return False

    async def circle_search(self):
        platform_np = mil_tools.rosmsg_to_numpy(self.waypoint_res.objects[0].position)
        await self.move.look_at(platform_np).set_position(platform_np).backward(
            self.circle_radius
        ).yaw_left(90, unit="deg").go(move_type="drive")

        done_circle = False

        async def do_circle():
            await self.move.circle_point(
                platform_np, direction=self.circle_direction
            ).go()
            done_circle = True  # noqa flake8 can't see that it is defined above

        await do_circle()
        while not done_circle:
            res = await self.get_any_shape()
            if res is False:
                await self.nh.sleep(0.25)
                continue
            fprint(
                "Shape ({}found, using normal to look at other 3 shapes if needed".format(
                    res[0]
                ),
                title="DETECT DELIVER",
                msg_color="green",
            )
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
            move_opposite_side = (
                self.move.set_position(point_opposite_side)
                .look_at(center_point)
                .yaw_left(90, unit="deg")
            )

            left_or_whatever_point = center_point + rotated_norm * self.circle_radius
            move_left_or_whatever = (
                self.move.set_position(left_or_whatever_point)
                .look_at(center_point)
                .yaw_left(90, unit="deg")
            )

            right_or_whatever_point = center_point - rotated_norm * self.circle_radius
            move_right_or_whatever = (
                self.move.set_position(right_or_whatever_point)
                .look_at(center_point)
                .yaw_left(90, unit="deg")
            )

            await self.search_sides(
                (move_right_or_whatever, move_opposite_side, move_left_or_whatever)
            )
            return
        fprint(
            "No shape found after complete circle",
            title="DETECT DELIVER",
            msg_color="red",
        )
        raise Exception("No shape found on platform")

    def update_shape(self, shape_res, normal_res, tf):
        self.identified_shapes[(shape_res.Shape, shape_res.Color)] = self.get_shape_pos(
            normal_res, tf
        )

    def correct_shape(self, tup):
        shape, color = tup
        return (self.Color == "ANY" or self.Color == color) and (
            self.Shape == "ANY" or self.Shape == shape
        )

    async def search_side(self):
        fprint("Searching side", title="DETECT DELIVER", msg_color="green")
        start_time = self.nh.get_time()
        while self.nh.get_time() - start_time < genpy.Duration(self.LOOK_AT_TIME):
            res = await self.get_any_shape()
            if res is not False:
                return res
            await self.nh.sleep(0.1)
        return False

    async def search_sides(self, moves):
        for move in moves:
            await move.go(move_type="drive")
            res = await self.search_side()
            if res is False:
                fprint(
                    "No shape found on side", title="DETECT DELIVER", msg_color="red"
                )
                continue
            shape_color, found_pose = res
            if self.correct_shape(shape_color):
                self.shape_pose = found_pose
                return
            fprint(
                "Saw (Shape={}, Color={}) on this side".format(
                    shape_color[0], shape_color[1]
                ),
                title="DETECT DELIVER",
                msg_color="green",
            )

    async def search_shape(self):
        shapes = await self.get_shape()
        if shapes.found:
            for shape in shapes.shapes.list:
                normal_res = await self.get_normal(shape)
                if normal_res.success:
                    enu_cam_tf = await self.tf_listener.get_transform(
                        "/enu", "/" + shape.header.frame_id, shape.header.stamp
                    )
                    if self.correct_shape(shape):
                        self.shape_pose = self.get_shape_pos(normal_res, enu_cam_tf)
                        return True
                    self.update_shape(shape, normal_res, enu_cam_tf)

                else:
                    if not self.last_lidar_error == normal_res.error:
                        fprint(
                            f"Normal not found Error={normal_res.error}",
                            title="DETECT DELIVER",
                            msg_color="red",
                        )
                    self.last_lidar_error = normal_res.error
        else:
            if not self.last_shape_error == shapes.error:
                fprint(
                    f"shape not found Error={shapes.error}",
                    title="DETECT DELIVER",
                    msg_color="red",
                )
            self.last_shape_error = shapes.error
        return False

    def select_backup_shape(self):
        for (shape, color), point_normal in self.identified_shapes.items():
            self.shape_pose = point_normal
            if self.Shape == shape or self.Color == color:
                fprint(
                    "Correct shape not found, resorting to shape={} color={}".format(
                        shape, color
                    ),
                    title="DETECT DELIVER",
                    msg_color="yellow",
                )
                return
        if self.shape_pose is None:
            raise Exception("None seen")
        fprint(
            "Correct shape not found, resorting to random shape",
            title="DETECT DELIVER",
            msg_color="yellow",
        )

    async def align_to_target(self):
        if self.shape_pose is None:
            self.select_backup_shape()
        goal_point, goal_orientation = self.get_aligned_pose(
            self.shape_pose[0], self.shape_pose[1]
        )
        move = (
            self.move.set_position(goal_point)
            .set_orientation(goal_orientation)
            .forward(self.target_offset_meters)
        )
        # Adjust for location of shooter
        move = move.left(-self.shooter_baselink_tf._p[1]).forward(
            -self.shooter_baselink_tf._p[0]
        )
        fprint(
            f"Aligning to shoot at {move}",
            title="DETECT DELIVER",
            msg_color="green",
        )
        move_complete = move.go(move_type="skid", blind=True)
        return move_complete

    def get_shape(self):
        return self.vision_proxies["get_shape"].get_response(Shape="ANY", Color="ANY")

    def get_aligned_pose(self, enupoint, enunormal):
        aligned_position = (
            enupoint + self.shoot_distance_meters * enunormal
        )  # moves x meters away
        angle = np.arctan2(-enunormal[0], enunormal[1])
        aligned_orientation = trns.quaternion_from_euler(
            0, 0, angle
        )  # Align perpendicular
        return (aligned_position, aligned_orientation)

    def get_shape_pos(self, normal_res, enu_cam_tf):
        enunormal = enu_cam_tf.transform_vector(
            mil_tools.rosmsg_to_numpy(normal_res.normal)
        )
        enupoint = enu_cam_tf.transform_point(
            mil_tools.rosmsg_to_numpy(normal_res.closest)
        )
        return (enupoint, enunormal)

    async def get_normal(self, shape):
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
        return normal_res

    def normal_is_sane(self, vector3):
        return abs(mil_tools.rosmsg_to_numpy(vector3)[1]) < 0.4

    async def shoot_all_balls(self):
        for i in range(self.NUM_BALLS):
            goal = await self.shooterLoad.send_goal(ShooterDoAction())
            fprint(
                f"Loading Shooter {i}",
                title="DETECT DELIVER",
                msg_color="green",
            )
            await goal.get_result()
            await self.nh.sleep(2)
            goal = await self.shooterFire.send_goal(ShooterDoAction())
            fprint(f"Firing Shooter {i}", title="DETECT DELIVER", msg_color="green")
            await goal.get_result()
            fprint(
                f"Waiting {self.WAIT_BETWEEN_SHOTS} seconds between shots",
                title="DETECT DELIVER",
                msg_color="green",
            )
            await self.nh.sleep(self.WAIT_BETWEEN_SHOTS)

    async def continuously_align(self):
        fprint("Starting Forest Align", title="DETECT DELIVER", msg_color="green")
        while True:
            shooter_pose = await self.shooter_pose_sub.get_next_message()
            if self.align_forest_pause:
                await self.nh.sleep(0.1)
                continue
            shooter_pose = shooter_pose.pose

            cen = np.array([shooter_pose.position.x, shooter_pose.position.y])
            yaw = trns.euler_from_quaternion(
                [
                    shooter_pose.orientation.x,
                    shooter_pose.orientation.y,
                    shooter_pose.orientation.z,
                    shooter_pose.orientation.w,
                ]
            )[2]
            q = trns.quaternion_from_euler(0, 0, yaw)
            p = np.append(cen, 0)
            # fprint("Forest Aligning to p=[{}] q=[{}]".format(p, q), title="DETECT DELIVER", msg_color='green')

            # Prepare move to follow shooter
            move = self.move.set_position(p).set_orientation(q).yaw_right(90, "deg")

            # Adjust move for location of target
            move = move.forward(self.target_offset_meters)

            # Adjust move for location of launcher
            move = move.left(-self.shooter_baselink_tf._p[1]).forward(
                -self.shooter_baselink_tf._p[0]
            )

            # Move away a fixed distance to make the shot
            move = move.left(self.shoot_distance_meters)

            await move.go(move_type="bypass")

    async def shoot_and_align_forest(self):
        move = await self.align_to_target()
        if move.failure_reason != "":
            fprint(
                "Error Aligning with target = {}. Ending mission :(".format(
                    move.failure_reason
                ),
                title="DETECT DELIVER",
                msg_color="red",
            )
            return
        fprint(
            "Aligned success. Shooting while using forest realign",
            title="DETECT DELIVER",
            msg_color="green",
        )
        align_defer = self.continuously_align()
        fprint(
            "Sleeping for {} seconds to allow for alignment",
            title=f"DETECT DELIVER",
            msg_color="green",
        )
        await self.nh.sleep(self.FOREST_SLEEP)
        for i in range(self.NUM_BALLS):
            goal = await self.shooterLoad.send_goal(ShooterDoAction())
            fprint(
                f"Loading Shooter {i}",
                title="DETECT DELIVER",
                msg_color="green",
            )
            await goal.get_result()
            await self.nh.sleep(2)
            self.align_forest_pause = True
            goal = await self.shooterFire.send_goal(ShooterDoAction())
            fprint(f"Firing Shooter {i}", title="DETECT DELIVER", msg_color="green")
            await goal.get_result()
            await self.nh.sleep(1)
            self.align_forest_pause = False
            fprint(
                f"Waiting {self.WAIT_BETWEEN_SHOTS} seconds between shots",
                title="DETECT DELIVER",
                msg_color="green",
            )
            await self.nh.sleep(self.WAIT_BETWEEN_SHOTS)
        align_defer.cancel()

    async def backup_from_target(self):
        await self.move.left(self.BACKUP_DISTANCE).go()

    async def shoot_and_align(self):
        move = await self.align_to_target()
        if move.failure_reason != "":
            fprint(
                "Error Aligning with target = {}. Ending mission :(".format(
                    move.failure_reason
                ),
                title="DETECT DELIVER",
                msg_color="red",
            )
            return
        fprint(
            "Aligned success. Shooting without realignment",
            title="DETECT DELIVER",
            msg_color="green",
        )
        await self.shoot_all_balls()

    async def setup_mission(self):
        stc_color = await self.mission_params["scan_the_code_color3"].get(
            raise_exception=False
        )
        if stc_color is False:
            color = "ANY"
        else:
            color = stc_color
        # color = "ANY"
        shape = "ANY"
        fprint(
            f"Setting search shape={shape} color={color}",
            title="DETECT DELIVER",
            msg_color="green",
        )
        await self.mission_params["detect_deliver_shape"].set(shape)
        await self.mission_params["detect_deliver_color"].set(color)

    async def run(self, parameters):
        await self.setup_mission()
        fprint("STARTING MISSION", title="DETECT DELIVER", msg_color="green")
        await self.vision_proxies["get_shape"].start()
        await self.set_shape_and_color()  # Get correct goal shape/color from params
        await self.get_waypoint()  # Get waypoint of shooter target
        await self.circle_search()  # Go to waypoint and circle until target found
        #  await self.shoot_and_align()      # Align to target and shoot
        await self.shoot_and_align_forest()  # Align to target and shoot
        await self.backup_from_target()
        await self.vision_proxies["get_shape"].stop()
        fprint("ENDING MISSION", title="DETECT DELIVER", msg_color="green")
