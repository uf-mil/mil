#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import os
import traceback
from typing import Any, Callable, Coroutine, Sequence

import genpy
import mil_ros_tools
import numpy as np
import rospkg
import sensor_msgs.point_cloud2 as pc2
import yaml
from axros import NodeHandle, ServiceClient, action, axros_tf, serviceclient, types
from mil_missions_core import BaseMission
from mil_msgs.msg import (
    MoveToAction,
    MoveToFeedback,
    MoveToGoal,
    MoveToResult,
    PoseTwistStamped,
    RangeStamped,
)
from mil_msgs.srv import (
    ObjectDBQuery,
    ObjectDBQueryRequest,
    SetGeometry,
    SetGeometryRequest,
)
from mil_passive_sonar.msg import ProcessedPing
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerRequest

# from subjugator_msgs.srv import SetValve, SetValveRequest
from sub8_actuator_board.srv import SetValve, SetValveRequest
from subjugator_msgs.srv import (
    VisionRequest,
    VisionRequest2D,
    VisionRequest2DRequest,
    VisionRequest2DResponse,
    VisionRequestRequest,
)
from tf.transformations import quaternion_from_euler, quaternion_multiply
from vision_msgs.msg import Detection2DArray

from . import exceptions, pose_editor


class VisionProxy:
    """
    General Interface for communicating with perception nodes.

    Make sure your perception nodes are structured as follows:
    * The node provides an enable service to start and stop percetion
    * The node provides a 3d and/or 2d pose service to get the position of the
      object of interest in real world or pixel
    * All services need to have the same root
      * For example, a buoy finder node may provide the following:

          .. code-block::

            /vision/buoy_finder/enable  # Starts and stops the perception (type = setBool)
            /vision/buoy_finder/2D      # Returns the pixel coordinates for the buoy (type = VisionRequest2D)
            /vision/buoy_finder/pose    # Returns a 3d pose of the buoy (type = VisionRequest)
    """

    _get_2d_service: ServiceClient
    _get_pose_service: ServiceClient
    _enable_service: ServiceClient
    _set_geometry_service: ServiceClient

    def __init__(self, service_root: str, nh: NodeHandle):
        assert "vision" in service_root, "expected 'vision' in the name of service_root"
        self._get_2d_service = nh.get_service_client(
            service_root + "/2D", VisionRequest2D
        )
        self._get_pose_service = nh.get_service_client(
            service_root + "/pose", VisionRequest
        )
        self._enable_service = nh.get_service_client(service_root + "/enable", SetBool)
        self._set_geometry_service = nh.get_service_client(
            service_root + "/set_geometry", SetGeometry
        )

    def start(self) -> Coroutine[Any, Any, types.Message]:
        """
        Allow user to start the vision processing backend.

        Can be used when the mission starts.
        """
        return self._enable_service(SetBoolRequest(data=True))

    def stop(self) -> Coroutine[Any, Any, types.Message]:
        """
        Allow user to stop the vision processing backend.

        Can be used after the mission completes.
        """
        return self._enable_service(SetBoolRequest(data=False))

    def get_2d(self, target: str = "") -> Coroutine[Any, Any, types.Message] | None:
        """
        Get the 2D projection of the thing.

        Camera deprojection stuff should be done elsewhere, this function is for when
        we don't know the depth of the thing we're targeting.
        """
        # TODO: Do something intelligent with the stamp
        #     - This is not "obviously" in any reference frame
        #     - We'll assume the user knows what they're doing
        #     - Determine rotation around Z and undo that?
        try:
            pose = self._get_2d_service(VisionRequest2DRequest(target_name=target))
        except serviceclient.ServiceError:
            return None
        return pose

    def get_pose(
        self, target: str = "", in_frame=None
    ) -> Coroutine[Any, Any, types.Message] | None:
        """
        Get the 3D pose of the object we're after.
        """
        # TODO:
        #     - Implement in_frame
        #     - Use the time information in the header
        try:
            pose = self._get_pose_service(VisionRequestRequest(target_name=target))
        except serviceclient.ServiceError:
            return None
        except Exception as e:
            print(type(e))
        return pose

    def set_geometry(
        self, polygon: SetGeometry
    ) -> Coroutine[Any, Any, types.Message] | None:
        try:
            res = self._set_geometry_service(SetGeometryRequest(model=polygon))
        except serviceclient.ServiceError:
            return None
        return res

    @classmethod
    def get_response_direction(
        cls, vision_response: VisionRequest2DResponse
    ) -> tuple[np.ndarray, float]:
        xy = np.array([vision_response.pose.x, vision_response.pose.y])
        bounds = np.array([[vision_response.max_x, vision_response.max_y]])
        theta = vision_response.pose.theta
        return (xy - (bounds / 2.0)) / bounds, theta


class _VisionProxies:
    """
    Gives interface to vision proxies.

    Add vision proxy names and roots to the yaml config file and access them through here, ex:

    .. code-block:: python

        >>> await sub.vision_proxies.buoy_finder.start()
        >>> pose_2d = await sub.vision_proxies.buoy_finder.get_2D('red')
    """

    def __init__(self, nh: NodeHandle, file_name: str):
        rospack = rospkg.RosPack()
        config_file = os.path.join(
            rospack.get_path("subjugator_missions"), "subjugator_missions", file_name
        )
        f = yaml.full_load(open(config_file))

        self.proxies: dict[str, VisionProxy] = {}
        for name, params in f.items():
            self.proxies[name] = VisionProxy(params["root"], nh)

    def __getattr__(self, proxy: str) -> VisionProxy | None:
        return self.proxies.get(proxy, None)


class _PoseProxy:
    def __init__(
        self,
        sub: SubjuGatorMission,
        pose: pose_editor.PoseEditor,
        print_only: bool = False,
    ):
        self._sub = sub
        self._pose = pose
        self.print_only = print_only

    # Normal moves get routed here
    def __getattr__(self, name: str) -> Callable:
        def sub_attr_proxy(*args, **kwargs):
            return _PoseProxy(
                self._sub,
                getattr(self._pose, name)(*args, **kwargs),
                print_only=self.print_only,
            )

        return sub_attr_proxy

    # Some special moves
    def to_height(self, height):
        dist_to_bot = self._sub._dvl_range_sub.get_last_message()
        delta_height = dist_to_bot.range - height
        return self.down(delta_height)

    def check_goal(self) -> None:
        """
        Check end goal for feasibility.

        Current checks are:
        * End goal can't be above the water
        """
        # End goal can't be above the water
        if self._pose.position[2] > 0:
            print("GOAL TOO HIGH")
            self._pos.position = -0.6

    async def go(self, *args, **kwargs):
        if self.print_only:
            print(self._pose)
            return self._sub.nh.sleep(0.1)

        self.check_goal()

        goal = self._sub._moveto_action_client.send_goal(
            self._pose.as_MoveToGoal(*args, **kwargs)
        )
        result = await goal.get_result()
        if result.error == "killed":
            raise exceptions.KilledException()
        return result

    def go_trajectory(self, *args, **kwargs):
        traj = self._sub._trajectory_pub.publish(
            self._pose.as_PoseTwistStamped(*args, **kwargs)
        )
        return traj


class _ActuatorProxy:
    """
    Wrapper for making service calls to pneumatic valve board.

    Example usage:
    TODO
    """

    _actuator_service: ServiceClient

    def __init__(self, nh: NodeHandle):
        self._actuator_service = nh.get_service_client("/set_valve", SetValve)
        self.nh = nh

    def open(self, id: int) -> Coroutine[Any, Any, types.Message]:
        return self.set(id, True)

    def close(self, id: int) -> Coroutine[Any, Any, types.Message]:
        return self.set(id, False)

    def set(self, id: int, opened: bool) -> Coroutine[Any, Any, types.Message]:
        return self._actuator_service(SetValveRequest(actuator=id, opened=opened))

    async def pulse(self, id: int, time: float = 0.5):
        await self.open(id)
        await self.nh.sleep(time)
        await self.close(id)
        return

    async def gripper_open(self):
        await self.close(4)
        await self.pulse(5)
        return

    async def gripper_close(self):
        await self.open(4)
        return

    async def shoot_torpedo1(self):
        await self.pulse(0)
        return

    async def shoot_torpedo2(self):
        await self.pulse(1)
        return

    async def drop_marker(self):
        await self.pulse(2, time=0.75)
        await self.pulse(3, time=0.25)
        return


class SubjuGatorMission(BaseMission):
    nh: NodeHandle
    _moveto_action_client: action.ActionClient[MoveToGoal, MoveToResult, MoveToFeedback]

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @classmethod
    async def setup_base(cls, mission_server):
        await super().setup_base(mission_server)
        cls._moveto_action_client = action.ActionClient(cls.nh, "moveto", MoveToAction)
        cls._odom_sub = cls.nh.subscribe("odom", Odometry)
        cls._trajectory_sub = cls.nh.subscribe("trajectory", PoseTwistStamped)
        cls._trajectory_pub = cls.nh.advertise("trajectory", PoseTwistStamped)
        cls._dvl_range_sub = cls.nh.subscribe("dvl/range", RangeStamped)
        cls._tf_listener = axros_tf.TransformListener(cls.nh)
        cls.vision_proxies = _VisionProxies(cls.nh, "vision_proxies.yaml")
        cls.actuators = _ActuatorProxy(cls.nh)
        cls.test_mode = False
        cls.pinger_sub = cls.nh.subscribe("/hydrophones/processed", ProcessedPing)
        cls.yolo_objects = cls.nh.subscribe("/yolov7/detections", Detection2DArray)

        await asyncio.gather(
            cls._moveto_action_client.setup(),
            cls._odom_sub.setup(),
            cls._trajectory_sub.setup(),
            cls._trajectory_pub.setup(),
            cls._dvl_range_sub.setup(),
            cls._tf_listener.setup(),
            cls.pinger_sub.setup(),
            cls.yolo_objects.setup(),
        )

    @classmethod
    async def shutdown_base(cls) -> None:
        await asyncio.gather(
            cls._moveto_action_client.shutdown(),
            cls._odom_sub.shutdown(),
            cls._trajectory_sub.shutdown(),
            cls._trajectory_pub.shutdown(),
            cls._dvl_range_sub.shutdown(),
            cls._tf_listener.shutdown(),
            cls.pinger_sub.shutdown(),
            cls.yolo_objects.shutdown(),
        )

    @property
    def pose(self) -> pose_editor.PoseEditor:
        last_odom_msg = self._odom_sub.get_last_message()
        if self.test_mode:
            last_odom_msg = Odometry()  # All 0's
        pose = pose_editor.PoseEditor.from_Odometry(last_odom_msg)
        return pose

    async def tx_pose(self):
        """
        Slightly safer to use.
        """
        if self.test_mode:
            await self.nh.sleep(0.1)
            blank = mil_ros_tools.pose_to_numpy(Odometry().pose.pose)
            return blank

        next_odom_msg = await self._odom_sub.get_next_message()
        pose = mil_ros_tools.pose_to_numpy(next_odom_msg.pose.pose)
        return pose

    @property
    def move(self) -> _PoseProxy:
        return _PoseProxy(self, self.pose, self.test_mode)

    async def get_dvl_range(self):
        msg = await self._dvl_range_sub.get_next_message()
        return msg.range

    async def get_in_frame(self, pose_stamped, frame: str = "map"):
        """
        TODO
        """
        transform = await self._tf_listener.get_transform(
            frame, pose_stamped.header.frame_id, pose_stamped.header.stamp
        )
        tft = axros_tf.Transform.from_Pose_message(pose_stamped.pose)
        full_transform = transform * tft
        position = np.array(full_transform._p)
        orientation = np.array(full_transform._q)

        return [position, orientation]


class Searcher:
    def __init__(self, sub: SubjuGatorMission, vision_proxy: Callable, search_pattern):
        """
        Give a sub_singleton, a function to call for the object you're looking for,
        and a list poses to execute in order to find it (can be a list of relative
        positions or pose_editor poses).
        """
        self.sub = sub
        self.vision_proxy = vision_proxy
        self.search_pattern = search_pattern

        self.object_found = False
        self.response = None

    async def start_search(
        self,
        timeout: float = 60,
        loop: bool = True,
        spotings_req: int = 2,
        speed: float = 0.1,
    ):
        print("SEARCHER - Starting.")
        looker = asyncio.create_task(self._run_look(spotings_req))
        searcher = asyncio.create_task(self._run_search_pattern(loop, speed))

        start_pose = self.sub.move.forward(0)
        start_time = self.sub.nh.get_time()
        while self.sub.nh.get_time() - start_time < genpy.Duration(timeout):
            # If we find the object
            if self.object_found:
                searcher.cancel()
                print("SEARCHER - Object found.")
                return self.response

            await self.sub.nh.sleep(0.1)

        print("SEARCHER - Object NOT found. Returning to start position.")
        looker.cancel()
        searcher.cancel()

        await start_pose.go()

    async def _run_search_pattern(self, loop: bool, speed: float):
        """
        Look around using the search pattern.

        If `loop` is true, then keep iterating over the list until timeout is
        reached or we find it.
        """
        print("SEARCHER - Executing search pattern.")
        try:
            if loop:
                while True:
                    for pose in self.search_pattern:
                        print("SEARCHER - going to next position.")
                        if isinstance(pose, list) or isinstance(pose, np.ndarray):
                            await self.sub.move.relative(pose).go(speed=speed)
                        else:
                            await pose.go()

                        await self.sub.nh.sleep(2)

            else:
                for pose in self.search_pattern:
                    if isinstance(pose, list) or isinstance(pose, np.ndarray):
                        await self.sub.move.relative(np.array(pose)).go(speed=speed)
                    else:
                        await pose.go()

                    await self.sub.nh.sleep(2)
        except asyncio.CancelledError:
            print("SEARCHER - Cancelling.")
        else:
            traceback.print_exc()

    async def _run_look(self, spotings_req: int):
        """
        Look for the object using the vision proxy.

        Only return true when we spotted the objects `spotings_req` many times
        (for false positives).
        """
        spotings = 0
        print("SEARCHER - Looking for object.")
        try:
            while True:
                resp = await self.vision_proxy()
                if resp.found:
                    print(f"SEARCHER - Object found! {spotings + 1}/{spotings_req}")
                    spotings += 1
                    if spotings >= spotings_req:
                        self.object_found = True
                        self.response = resp
                        break
                else:
                    spotings = 0

                await self.sub.nh.sleep(0.5)
        except asyncio.CancelledError:
            print("SEARCHER - Cancelling.")
        else:
            traceback.print_exc()


class PoseSequenceCommander:
    def __init__(self, sub: SubjuGatorMission):
        self.sub = sub

    async def go_to_sequence_eulers(
        self,
        positions: Sequence[Sequence[float]],
        orientations: Sequence[Sequence[float]],
        speed: float = 0.2,
    ):
        """
        Pass a list of positions and orientations (euler).
        Each is realive to the sub's pose following the previous
        pose command.
        """
        for i in range(len(positions)):
            await self.sub.move.look_at_without_pitching(
                np.array(positions[i][0:3])
            ).go(speed)
            await self.sub.move.relative(np.array(positions[i][0:3])).go(speed)
            await self.sub.move.set_orientation(
                quaternion_multiply(
                    self.sub.pose.orientation,
                    quaternion_from_euler(
                        orientations[i][0], orientations[i][1], orientations[i][2]
                    ),
                )
            ).go(speed)

    async def go_to_sequence_quaternions(
        self,
        positions: Sequence[Sequence[float]],
        orientations: Sequence[Sequence[float]],
        speed: float = 0.2,
    ):
        """
        Pass a list of positions and orientations (quaternion).
        Each is realive to the sub's pose following the previous
        pose command.
        """
        for i in range(len(positions)):
            await self.sub.move.look_at_without_pitching(
                np.array(positions[i][0:3])
            ).go(speed)
            await self.sub.move.relative(np.array(positions[i][0:3])).go(speed)
            await self.sub.move.set_orientation(
                quaternion_multiply(
                    self.sub.pose.orientation,
                    (
                        orientations[i][0],
                        orientations[i][1],
                        orientations[i][2],
                        orientations[i][3],
                    ),
                )
            ).go(speed)


class SonarObjects:
    _clear_pcl: ServiceClient
    _objects_service: ServiceClient

    def __init__(self, sub: SubjuGatorMission, pattern=None):
        """
        SonarObjects: a helper to search and find objects

        Everything must be in map frame

        Parameters:
        sub: the sub object
        pattern: an array of pose goals (i.e: [sub.move.forward(1)])
        """
        self.sub = sub
        if pattern is None:
            self.pattern = [sub.move.forward(0)]
        self.pattern = pattern
        self._clear_pcl = self.sub.nh.get_service_client(
            "/ogrid_pointcloud/clear_pcl", Trigger
        )

        self._objects_service = self.sub.nh.get_service_client(
            "/ogrid_pointcloud/get_objects", ObjectDBQuery
        )

    def __del__(self):
        print("cleared SonarObject -- thanks TX")

    async def start_search(self, speed: float = 0.5, clear: bool = False):
        """
        Do a search and return all objects.

        Parameters:
        speed: how fast sub should move
        clear: clear pointcloud
        """
        if clear:
            print("SONAR_OBJECTS: clearing pointcloud")
            await self._clear_pcl(TriggerRequest())

        print("SONAR_OBJECTS: running pattern")
        await self._run_pattern(speed)

        print("SONAR_OBJECTS: requesting objects")
        res = await self._objects_service(ObjectDBQueryRequest())
        return res

    async def start_search_in_cone(
        self,
        start_point,
        ray,
        angle_tol: float = 30,
        distance_tol: float = 10,
        speed: float = 0.5,
        clear: bool = False,
        c_func: Callable = None,
    ):
        if clear:
            print("SONAR_OBJECTS: clearing pointcloud")
            await self._clear_pcl(TriggerRequest())

        await self.sub.nh.sleep(1)

        for pose in self.pattern:
            await pose.go(speed=speed, blind=True)
            # sleep
            await self.sub.nh.sleep(0.1)

            # Break out of loop if we find something satisfying function
            res = await self._objects_service(ObjectDBQueryRequest())
            g_obj = self._get_objects_within_cone(
                res.objects, start_point, ray, angle_tol, distance_tol
            )
            g_obj = self._sort_by_angle(g_obj, ray, start_point)

            if c_func is not None:
                out = c_func(g_obj, ray)
                print("SONAR_OBJECTS: " + str(out))
                if out is not None or out is True:
                    print("SONAR_OBJECTS: found objects satisfying function")
                    break

        res = await self._objects_service(ObjectDBQueryRequest())
        g_obj = self._get_objects_within_cone(
            res.objects, start_point, ray, angle_tol, distance_tol
        )
        g_obj = self._sort_by_angle(g_obj, ray, start_point)
        res.objects = g_obj
        return res

    async def start_until_found_x(
        self, speed: float = 0.5, clear: bool = False, object_count: int = 0
    ):
        """
        Search until a number of objects are found.

        Parameters:
        speed: how fast sub should move
        clear: clear pointcloud
        object_count: how many objects we want
        """
        if clear:
            print("SONAR_OBJECTS: clearing pointcloud")
            await self._clear_pcl(TriggerRequest())
        count = -1
        while count < object_count:
            for pose in self.pattern:
                await pose.go(speed=speed)
                res = await self._objects_service(ObjectDBQueryRequest())
                count = len(res.objects)
                if count >= object_count:
                    return res
        return None

    async def start_until_found_in_cone(
        self,
        start_point: np.ndarray,
        speed: float = 0.5,
        clear: bool = False,
        object_count: int = 0,
        ray: np.ndarray = np.array([0, 1, 0]),
        angle_tol: float = 30,
        distance_tol: float = 12,
    ):
        """
        Search until objects are found within a cone-shaped range

        Args:
            start_point: numpy array for the starting point of the direction vector
            speed: how fast the sub should move
            clear: should the pointcloud be clear beforehand
            object_count: how many objects we are looking for
            ray: the direction vector
            angle_tol: how far off the direction vector should be allowed
            distance_tol: how far away are we willing to accept

        Returns:
            ObjectDBQuery: with objects field filled by good objects
        """
        if clear:
            print("SONAR_OBJECTS: clearing pointcloud")
            await self._clear_pcl(TriggerRequest())
        count = -1
        while count < object_count:
            for pose in self.pattern:
                await pose.go(speed=speed, blind=True)
                res = await self._objects_service(ObjectDBQueryRequest())
                g_obj = self._get_objects_within_cone(
                    res.objects, start_point, ray, angle_tol, distance_tol
                )
                if g_obj is None:
                    continue
                count = len(g_obj)
                print(f"SONAR OBJECTS: found {count} that satisfy cone")
                if count >= object_count:
                    g_obj = self._sort_by_angle(g_obj, ray, start_point)
                    res.objects = g_obj
                    return res
        return None

    @staticmethod
    def _get_objects_within_cone(
        objects,
        start_point: np.ndarray,
        ray: np.ndarray,
        angle_tol: float,
        distance_tol: float,
    ):
        ray = ray / np.linalg.norm(ray)
        out = []
        for o in objects:
            print("=" * 50)
            pos = mil_ros_tools.rosmsg_to_numpy(o.pose.position)
            print(f"pos {pos}")
            dist = np.dot(pos - start_point, ray)
            print(f"dist {dist}")
            if dist > distance_tol or dist < 0:
                continue
            vec_for_pos = pos - start_point
            vec_for_pos = vec_for_pos / np.linalg.norm(vec_for_pos)
            angle = np.arccos(vec_for_pos.dot(ray)) * 180 / np.pi
            print(f"angle {angle}")
            if angle > angle_tol:
                continue
            out.append(o)
        return out

    @staticmethod
    def _sort_by_angle(objects, ray: np.ndarray, start_point: np.ndarray):
        """
        _sort_by_angle: returns object list sorted by angle

        Parameters:
            objects:
            ray: directional unit vector
            start_point: base point for vector in map
        """
        positions = [mil_ros_tools.rosmsg_to_numpy(o.pose.position) for o in objects]
        dots = [(p / np.linalg.norm(p) - start_point).dot(ray) for p in positions]
        idx = np.argsort(dots)
        return np.array(objects)[idx]

    async def _run_pattern(self, speed: float):
        for pose in self.pattern:
            await pose.go(speed=speed)


class SonarPointcloud:
    def __init__(self, sub: SubjuGatorMission, pattern=None):
        if pattern is None:
            pattern = (
                [sub.move.zero_roll_and_pitch()]
                + [sub.move.pitch_down_deg(5)] * 5
                + [sub.move.zero_roll_and_pitch()]
            )
        self.sub = sub
        self.pointcloud = None
        self.pattern = pattern

    async def start(self, speed: float = 0.2):
        self._plane_subscriber = self.sub.nh.subscribe(
            "/ogrid_pointcloud/point_cloud/plane", PointCloud2
        )
        await self._plane_subscriber.setup()
        await self._run_move_pattern(speed)

        pc_gen = np.asarray(
            list(
                pc2.read_points(
                    self.pointcloud, skip_nans=True, field_names=("x", "y", "z")
                )
            )
        )
        return pc_gen

    async def _cat_newcloud(self):
        data = await self._plane_subscriber.get_next_message()
        if self.pointcloud is None:
            self.pointcloud = data
        else:
            gen = list(
                pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
            )
            pc_gen = list(
                pc2.read_points(
                    self.pointcloud, skip_nans=True, field_names=("x", "y", "z")
                )
            )
            concat = np.asarray(gen + pc_gen, np.float32)
            print(f"SONAR_POINTCLOUD - current size: {concat.shape}")
            self.pointcloud = mil_ros_tools.numpy_to_pointcloud2(concat)

    async def _run_move_pattern(self, speed: float):
        for pose in self.pattern:
            if isinstance(pose, list) or isinstance(pose, np.ndarray):
                await self.sub.move.relative(np.array(pose)).go(speed=speed)
            else:
                await pose.go()
            await self._cat_newcloud()

            await self.sub.nh.sleep(2)
