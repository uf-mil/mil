#!/usr/bin/env python3
import asyncio
import math

import axros
from axros import NodeHandle, axros_tf
from geographic_msgs.msg import GeoPath, GeoPoseStamped
from mil_msgs.srv import ObjectDBQuery
from mil_tools import numpy_to_point, rosmsg_to_numpy
from navigator_missions import NaviGatorMission
from navigator_msgs.srv import (
    AcousticBeacon,
    ChooseAnimal,
    MoveToWaypoint,
    MoveToWaypointRequest,
    TwoClosestCones,
)
from robot_localization.srv import FromLL, FromLLRequest, ToLL, ToLLRequest
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Empty, Float64, Float64MultiArray
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray
from vrx_gazebo.msg import Task
from vrx_gazebo.srv import ColorSequence

___author___ = "Kevin Allen"


class Vrx(NaviGatorMission):
    nh: NodeHandle

    @classmethod
    async def setup(cls):
        if hasattr(cls, "_cls_init"):
            return
        cls.from_lla = cls.nh.get_service_client("/fromLL", FromLL)
        cls.to_lla = cls.nh.get_service_client("/toLL", ToLL)
        cls.task_info_sub = cls.nh.subscribe("/vrx/task/info", Task)
        cls.scan_dock_color_sequence = cls.nh.get_service_client(
            "/vrx/scan_dock_deliver/color_sequence",
            ColorSequence,
        )
        cls.fire_ball = cls.nh.advertise("/wamv/shooters/ball_shooter/fire", Empty)
        cls.station_keep_goal = cls.nh.subscribe(
            "/vrx/station_keeping/goal",
            GeoPoseStamped,
        )
        cls.wayfinding_path_sub = cls.nh.subscribe("/vrx/wayfinding/waypoints", GeoPath)
        cls.station_keeping_pose_error = cls.nh.subscribe(
            "/vrx/station_keeping/pose_error",
            Float64,
        )
        cls.station_keeping_rms_error = cls.nh.subscribe(
            "/vrx/station_keeping/rms_error",
            Float64,
        )
        cls.wayfinding_min_errors = cls.nh.subscribe(
            "/vrx/wayfinding/min_errors",
            Float64MultiArray,
        )
        cls.wayfinding_mean_error = cls.nh.subscribe(
            "/vrx/wayfinding/mean_error",
            Float64,
        )
        cls.perception_landmark = cls.nh.advertise(
            "/vrx/perception/landmark",
            GeoPoseStamped,
        )
        await asyncio.gather(
            cls.task_info_sub.setup(),
            cls.fire_ball.setup(),
            cls.station_keep_goal.setup(),
            cls.wayfinding_path_sub.setup(),
            cls.station_keeping_pose_error.setup(),
            cls.station_keeping_rms_error.setup(),
            cls.wayfinding_min_errors.setup(),
            cls.wayfinding_mean_error.setup(),
            cls.perception_landmark.setup(),
        )

        cls.animal_landmarks = cls.nh.subscribe("/vrx/wildlife/animals/poses", GeoPath)
        cls.beacon_landmark = cls.nh.get_service_client("beaconLocator", AcousticBeacon)
        cls.circle_animal = cls.nh.get_service_client("/choose_animal", ChooseAnimal)
        cls.set_long_waypoint = cls.nh.get_service_client(
            "/set_long_waypoint",
            MoveToWaypoint,
        )
        cls.yolo_objects = cls.nh.subscribe("/yolov7/detections", Detection2DArray)
        cls.tf_listener = axros_tf.TransformListener(cls.nh)
        await cls.tf_listener.setup()
        cls.database_response = cls.nh.get_service_client(
            "/database/requests",
            ObjectDBQuery,
        )
        cls.get_two_closest_cones = cls.nh.get_service_client(
            "/get_two_closest_cones",
            TwoClosestCones,
        )
        await asyncio.gather(
            cls.animal_landmarks.setup(),
            cls.yolo_objects.setup(),
        )

        cls.pcodar_reset = cls.nh.get_service_client("/pcodar/reset", Trigger)

        cls.front_left_camera_info_sub = None
        cls.front_left_camera_sub = None
        cls.front_right_camera_info_sub = None
        cls.front_right_camera_sub = None

        cls._cls_init = True

    @classmethod
    async def shutdown(cls):
        return
        await asyncio.gather(
            cls.task_info_sub.shutdown(),
            cls.animal_landmarks.shutdown(),
            cls.yolo_objects.shutdown(),
            cls.fire_ball.shutdown(),
            cls.station_keep_goal.shutdown(),
            cls.wayfinding_path_sub.shutdown(),
            cls.station_keeping_pose_error.shutdown(),
            cls.station_keeping_rms_error.shutdown(),
            cls.wayfinding_min_errors.shutdown(),
            cls.wayfinding_mean_error.shutdown(),
            cls.perception_landmark.shutdown(),
        )

    def cleanup(self):
        pass

    @classmethod
    async def init_front_left_camera(cls):
        if cls.front_left_camera_sub is None:
            cls.front_left_camera_sub = cls.nh.subscribe(
                "/wamv/sensors/cameras/front_left_camera/image_raw",
                Image,
            )

        if cls.front_left_camera_info_sub is None:
            cls.front_left_camera_info_sub = cls.nh.subscribe(
                "/wamv/sensors/cameras/front_left_camera/camera_info",
                CameraInfo,
            )

        await asyncio.gather(
            cls.front_left_camera_sub.setup(),
            cls.front_left_camera_info_sub.setup(),
        )

    @classmethod
    async def init_front_right_camera(cls):
        if cls.front_right_camera_sub is None:
            cls.front_right_camera_sub = cls.nh.subscribe(
                "/wamv/sensors/cameras/front_right_camera/image_raw",
                Image,
            )

        if cls.front_right_camera_info_sub is None:
            cls.front_right_camera_info_sub = cls.nh.subscribe(
                "/wamv/sensors/cameras/front_right_camera/camera_info",
                CameraInfo,
            )

        await asyncio.gather(
            cls.front_right_camera_sub.setup(),
            cls.front_right_camera_info_sub.setup(),
        )

    async def geo_pose_to_enu_pose(self, geo):
        self.send_feedback("Waiting for LLA conversion")
        enu_msg = await self.from_lla(FromLLRequest(ll_point=geo.position))
        position_enu = rosmsg_to_numpy(enu_msg.map_point)
        orientation_enu = rosmsg_to_numpy(geo.orientation)
        return (position_enu, orientation_enu)

    async def enu_position_to_geo_point(self, enu_array):
        self.send_feedback("Waiting for LLA conversion")
        lla_msg = await self.to_lla(ToLLRequest(map_point=numpy_to_point(enu_array)))
        return lla_msg.ll_point

    async def get_latching_msg(self, sub: axros.Subscriber):
        msg = sub.get_last_message()
        if msg is None:
            msg = await sub.get_next_message()
        return msg

    async def wait_for_task_such_that(self, f):
        while True:
            msg = await self.task_info_sub.get_next_message()
            if f(msg):
                return None

    async def point_at_goal(self, goal_pos):
        vect = [goal_pos[0] - self.pose[0][0], goal_pos[1] - self.pose[0][1]]
        theta = math.atan2(vect[1], vect[0])
        orientation_fix = axros_tf.transformations.quaternion_from_euler(0, 0, theta)
        await self.move.set_orientation(orientation_fix).go(blind=True)

    async def send_trajectory_without_path(self, goal_pose):
        req = MoveToWaypointRequest()
        req.target_p.position.x = goal_pose[0][0]
        req.target_p.position.y = goal_pose[0][1]
        req.target_p.position.z = goal_pose[0][2]
        req.target_p.orientation.x = goal_pose[1][0]
        req.target_p.orientation.y = goal_pose[1][1]
        req.target_p.orientation.z = goal_pose[1][2]
        req.target_p.orientation.w = goal_pose[1][3]
        await self.set_long_waypoint(req)

    async def get_closest(self):
        await self.get_sorted_objects("all")

    async def reset_pcodar(self):
        await self.pcodar_reset(Trigger())

    async def run(self, parameters):
        await self.set_vrx_classifier_enabled.wait_for_service()
        await self._pcodar_set_params.wait_for_service()
        msg = await self.task_info_sub.get_next_message()
        task_name = msg.name
        if task_name == "station_keeping":
            await self.run_submission("VrxStationKeeping2")
        elif task_name == "wayfinding":
            await self.run_submission("VrxWayfinding2")
        elif task_name == "gymkhana":
            await self.run_submission("Gymkhana")
        elif task_name == "perception":
            await self.run_submission("VrxPerception")
        elif task_name == "wildlife":
            await self.run_submission("VrxWildlife")
        elif task_name == "scan_dock_deliver":
            await self.run_submission("ScanAndDock")
        msg = await self.task_info_sub.get_next_message()
        return msg
