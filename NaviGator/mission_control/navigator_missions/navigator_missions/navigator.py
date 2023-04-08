#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import os
from typing import Callable

import genpy
import mil_tools
import navigator_msgs.srv as navigator_srvs
import numpy as np
import rospkg
import uvloop
import yaml
from axros import NodeHandle, ROSMasterError, ServiceClient, action, axros_tf, util
from dynamic_reconfigure.msg import Config
from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
from geometry_msgs.msg import PointStamped, PoseStamped
from mil_misc_tools.text_effects import fprint
from mil_missions_core import BaseMission
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from mil_passive_sonar import TxHydrophonesClient
from mil_pneumatic_actuator.srv import SetValve, SetValveRequest
from mil_poi import TxPOIClient
from nav_msgs.msg import Odometry
from navigator_path_planner.msg import MoveAction, MoveGoal
from navigator_tools import MissingPerceptionObject
from roboteq_msgs.msg import Command
from ros_alarms import TxAlarmListener
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from std_srvs.srv import (
    SetBool,
    SetBoolRequest,
    SetBoolResponse,
    Trigger,
    TriggerRequest,
)
from topic_tools.srv import MuxSelect, MuxSelectRequest
from vision_msgs.msg import Detection2DArray

from .pose_editor import PoseEditor2


class MissionResult:
    NoResponse = 0
    ParamNotFound = 1
    DbObjectNotFound = 2
    OtherResponse = 100

    def __init__(
        self,
        success: bool = True,
        response=None,
        message: str = "",
        need_rerun: bool = False,
        post_function=None,
    ):
        self.success = success
        self.response = self.NoResponse if response is None else response
        self.message = message
        self.need_rerun = need_rerun
        self.post_function = lambda: None if post_function is None else post_function

    def __repr__(self):
        cool_bars = "=" * 75
        _pass = (
            cool_bars,
            "    Mission Success!",
            f"    Message: {self.message}",
            cool_bars,
        )
        _fail = (
            cool_bars,
            "    Mission Failure!",
            f"    Message: {self.message}",
            f"    Post function: {self.post_function.__name__}",
            cool_bars,
        )

        return "\n".join(_pass if self.success else _fail)


class NaviGatorMission(BaseMission):
    circle = "CIRCLE"
    cross = "CROSS"
    triangle = "TRIANGLE"
    red = "RED"
    green = "GREEN"
    blue = "BLUE"
    net_stc_results = None
    net_entrance_results = None
    max_grinch_effort = 500

    nh: NodeHandle

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @classmethod
    async def setup_base(cls, mission_runner):
        await super().setup_base(mission_runner)

        try:
            cls.is_vrx = await cls.nh.get_param("/is_vrx")
        except ROSMasterError:
            cls.is_vrx = False

        cls._moveto_client = action.ActionClient(cls.nh, "move_to", MoveAction)
        await cls._moveto_client.setup()

        # For missions to access clicked points / poses
        cls.rviz_goal = cls.nh.subscribe("/move_base_simple/goal", PoseStamped)
        cls.rviz_point = cls.nh.subscribe("/clicked_point", PointStamped)
        await asyncio.gather(cls.rviz_goal.setup(), cls.rviz_point.setup())

        cls._change_wrench = cls.nh.get_service_client("/wrench/select", MuxSelect)
        cls._change_trajectory = cls.nh.get_service_client(
            "/trajectory/select",
            MuxSelect,
        )
        cls._database_query = cls.nh.get_service_client(
            "/database/requests",
            ObjectDBQuery,
        )
        cls._reset_pcodar = cls.nh.get_service_client("/pcodar/reset", Trigger)
        cls._pcodar_set_params = cls.nh.get_service_client(
            "/pcodar/set_parameters",
            Reconfigure,
        )

        cls.pose = None

        def odom_set(odom: Odometry):
            return setattr(cls, "pose", mil_tools.odometry_to_numpy(odom)[0])

        cls._odom_sub = cls.nh.subscribe("odom", Odometry, odom_set)
        await cls._odom_sub.setup()

        if cls.is_vrx:
            cls._setup_vrx()
        else:
            await cls._setup_not_vrx()

    @classmethod
    async def shutdown_base(cls):
        await asyncio.gather(
            cls._moveto_client.shutdown(),
            cls.rviz_goal.shutdown(),
            cls.rviz_point.shutdown(),
            cls._odom_sub.shutdown(),
        )
        if not cls.is_vrx:
            await cls._shutdown_not_vrx()

    @classmethod
    def _setup_vrx(cls):
        cls.killed = False
        cls.odom_loss = False
        cls.set_vrx_classifier_enabled = cls.nh.get_service_client(
            "/vrx_classifier/set_enabled",
            SetBool,
        )

    @classmethod
    async def _setup_not_vrx(cls):
        cls.vision_proxies = {}
        cls._load_vision_services()

        cls.launcher_state = "inactive"
        cls._actuator_timing = await cls.nh.get_param("~actuator_timing")

        cls.mission_params = {}
        cls._load_mission_params()

        # If you don't want to use axros
        cls.ecef_pose = None

        cls.killed = "?"
        cls.odom_loss = "?"

        if await cls.nh.has_param("/is_simulation"):
            cls.sim = await cls.nh.get_param("/is_simulation")
        else:
            cls.sim = False

        def enu_odom_set(odom):
            return setattr(cls, "ecef_pose", mil_tools.odometry_to_numpy(odom)[0])

        cls._ecef_odom_sub = cls.nh.subscribe("absodom", Odometry, enu_odom_set)
        await cls._ecef_odom_sub.setup()

        cls.hydrophones = TxHydrophonesClient(cls.nh)

        cls.poi = TxPOIClient(cls.nh)
        await cls.poi.setup()

        cls._grinch_lower_time = await cls.nh.get_param("~grinch_lower_time")
        cls._grinch_raise_time = await cls.nh.get_param("~grinch_raise_time")
        cls.grinch_limit_switch_pressed = False
        cls._grinch_limit_switch_sub = cls.nh.subscribe(
            "/limit_switch",
            Bool,
            cls._grinch_limit_switch_cb,
        )
        cls._winch_motor_pub = cls.nh.advertise("/grinch_winch/cmd", Command)
        cls._grind_motor_pub = cls.nh.advertise("/grinch_spin/cmd", Command)
        await asyncio.gather(
            cls._grinch_limit_switch_sub.setup(),
            cls._winch_motor_pub.setup(),
            cls._grind_motor_pub.setup(),
        )

        try:
            cls._actuator_client = cls.nh.get_service_client(
                "/actuator_driver/actuate",
                SetValve,
            )
            cls._camera_database_query = cls.nh.get_service_client(
                "/camera_database/requests",
                navigator_srvs.CameraDBQuery,
            )
            cls._change_wrench = cls.nh.get_service_client("/wrench/select", MuxSelect)
            cls._change_trajectory = cls.nh.get_service_client(
                "/trajectory/select",
                MuxSelect,
            )
        except AttributeError as err:
            fprint(
                f"Error getting service clients in nav singleton init: {err}",
                title="NAVIGATOR",
                msg_color="red",
            )

        cls.tf_listener = axros_tf.TransformListener(cls.nh)
        await cls.tf_listener.setup()

        # Vision
        cls.set_classifier_enabled = cls.nh.get_service_client(
            "/classifier/set_enabled",
            SetBool,
        )
        cls.obstacle_course_vision_enable = cls.nh.get_service_client(
            "/vision/obsc/enable",
            SetBool,
        )
        cls.docks_vision_enable = cls.nh.get_service_client(
            "/vision/docks/enable",
            SetBool,
        )

        await cls._make_alarms()

        # We want to make sure odom is working before we continue
        fprint("Action client do you await?", title="NAVIGATOR")
        await util.wrap_time_notice(
            cls._moveto_client.wait_for_server(),
            2,
            "Lqrrt action server",
        )
        fprint("Yes he await!", title="NAVIGATOR")

        fprint("Waiting for odom...", title="NAVIGATOR")
        await util.wrap_time_notice(
            cls._odom_sub.get_next_message(),
            2,
            "Odom listener",
        )

        if not cls.sim:
            await util.wrap_time_notice(
                cls._ecef_odom_sub.get_next_message(),
                2,
                "ENU Odom listener",
            )
        print("Odom has been received!")

        cls.docking_scan = "NA"

        cls.front_left_camera_info_sub = None
        cls.front_left_camera_sub = None
        await cls.init_front_left_camera()
        cls.front_right_camera_info_sub = None
        cls.front_right_camera_sub = None
        await cls.init_front_right_camera()

        cls.yolo_objects = cls.nh.subscribe(
            "/yolov7/detections_model1",
            Detection2DArray,
        )
        cls.stc_objects = cls.nh.subscribe(
            "/yolov7/stc_detections_model",
            Detection2DArray,
        )
        await cls.yolo_objects.setup()
        await cls.stc_objects.setup()

    @classmethod
    async def _shutdown_not_vrx(cls):
        await asyncio.gather(
            cls._ecef_odom_sub.shutdown(),
            cls._grinch_limit_switch_sub.shutdown(),
            cls._winch_motor_pub.shutdown(),
            cls._grind_motor_pub.shutdown(),
            cls.tf_listener.shutdown(),
            cls.kill_listener.shutdown(),
            cls.poi.shutdown(),
            cls.front_left_camera_sub.shutdown(),
            cls.front_left_camera_info_sub.shutdown(),
            cls.front_right_camera_sub.shutdown(),
            cls.front_right_camera_info_sub.shutdown(),
            cls.yolo_objects.shutdown(),
            cls.stc_objects.shutdown(),
        )

    @classmethod
    async def init_front_left_camera(cls):
        if cls.front_left_camera_sub is None:
            cls.front_left_camera_sub = cls.nh.subscribe(
                "/wamv/sensors/camera/front_left_cam/image_raw",
                Image,
            )

        if cls.front_left_camera_info_sub is None:
            cls.front_left_camera_info_sub = cls.nh.subscribe(
                "/wamv/sensors/camera/front_left_cam/camera_info",
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
                "/wamv/sensors/camera/front_right_cam/image_raw",
                Image,
            )

        if cls.front_right_camera_info_sub is None:
            cls.front_right_camera_info_sub = cls.nh.subscribe(
                "/wamv/sensors/camera/front_right_cam/camera_info",
                CameraInfo,
            )

        await asyncio.gather(
            cls.front_right_camera_sub.setup(),
            cls.front_right_camera_info_sub.setup(),
        )

    @classmethod
    async def reset_pcodar(cls):
        res = await cls._reset_pcodar(TriggerRequest())
        return res

    @classmethod
    async def pcodar_set_params(self, **kwargs):
        result = await self._pcodar_set_params(ReconfigureRequest(Config(**kwargs)))
        return result

    @classmethod
    async def pcodar_label(cls, idx: int, name: str):
        cmd = "%d=%s" % (idx, name)
        await cls._database_query(ObjectDBQueryRequest(name="", cmd=cmd))

    @classmethod
    def _grinch_limit_switch_cb(cls, data):
        cls.grinch_limit_switch_pressed = data.data

    async def tx_pose(self):
        last_odom_msg = await self._odom_sub.get_next_message()
        return mil_tools.odometry_to_numpy(last_odom_msg)[0]

    @property
    async def tx_ecef_pose(self):
        last_odom_msg = await self._ecef_odom_sub.get_next_message()
        return mil_tools.odometry_to_numpy(last_odom_msg)[0]

    @property
    def move(self) -> PoseEditor2:
        return PoseEditor2(self, self.pose)

    def hold(self) -> action.GoalManager:
        goal = MoveGoal(move_type="hold")
        return self._moveto_client.send_goal(goal)

    def fetch_result(self, *args, **kwargs):
        # For a unified result class
        return MissionResult(*args, **kwargs)

    async def set_vision_dock(self):
        await self.obstacle_course_vision_enable(SetBoolRequest(data=False))
        await self.docks_vision_enable(SetBoolRequest(data=True))

    async def set_vision_obstacle_course(self):
        await self.docks_vision_enable(SetBoolRequest(data=False))
        await self.obstacle_course_vision_enable(SetBoolRequest(data=True))

    async def set_vision_off(self):
        await self.obstacle_course_vision_enable(SetBoolRequest(data=False))
        await self.docks_vision_enable(SetBoolRequest(data=False))

    async def spin_grinch(self, speed: float = 1.0, interval: float = 0.1):
        """
        Spin the grinch mechnaism. To avoid watchdog timeout, this is sent
        in a loop at the specified interface. So to stop spinning,
        cancel the defer

        Example:
        # start spinning
        d = self.spin_grinch()
        # do some stuf....
        ...
        # Stop spinning
        d.cancel()
        """
        while True:
            self._grind_motor_pub.publish(
                Command(setpoint=speed * self.max_grinch_effort),
            )
            await self.nh.sleep(interval)

    async def spin_winch(self, speed: float = -1.0, interval: float = 0.1):
        """
        Spin the grinch raise/lower mechanism. To avoid watchdog timeout, this is sent
        in a loop at the specified interface. So to stop spinning,
        cancel the defer
        """
        while True:
            self._winch_motor_pub.publish(
                Command(setpoint=speed * self.max_grinch_effort),
            )
            await self.nh.sleep(interval)

    async def deploy_grinch(self):
        """
        Deploy the grinch mechanism
        """
        winch_defer = asyncio.create_task(self.spin_winch(speed=1.0))
        await self.nh.sleep(self._grinch_lower_time)
        self._winch_motor_pub.publish(Command(setpoint=0))
        winch_defer.cancel()

    async def retract_grinch(self):
        """
        Retract the grinch mechanism
        """
        now = self.nh.get_time()
        end = now + genpy.Duration(self._grinch_raise_time)
        winch_defer = asyncio.create_task(self.spin_winch(speed=-1.0))
        while True:
            now = await self.nh.get_time()
            if self.grinch_limit_switch_pressed:
                print("limit switch pressed, stopping")
                break
            elif now >= end:
                print("retract timed out")
                break
            await self.nh.sleep(0.1)
        winch_defer.cancel()
        self._winch_motor_pub.publish(Command(setpoint=0))

    async def deploy_thruster(self, name: str):
        """
        Execute sequence to deploy one thruster
        """
        extend = name + "_extend"
        retract = name + "_retract"
        unlock = name + "_unlock"
        # Pull thruster up a bit to remove pressure from lock
        await self.set_valve(retract, True)
        await self.nh.sleep(self._actuator_timing["deploy_loosen_time"])
        # Stop pulling thruster up and unlock
        await self.set_valve(retract, False)
        await self.set_valve(unlock, True)
        # Begging extending piston to push thruster down
        await self.set_valve(extend, True)
        await self.nh.sleep(self._actuator_timing["deploy_wait_time"])
        # Lock and stop extending after waiting a time for lock to engage
        await self.set_valve(unlock, False)
        await self.nh.sleep(self._actuator_timing["deploy_lock_time"])
        await self.set_valve(extend, False)

    async def retract_thruster(self, name: str):
        """
        Execute sequence to retract one thruster
        """
        retract = name + "_retract"
        unlock = name + "_unlock"
        # Unlock and begin pulling thruster up
        await self.set_valve(unlock, True)
        await self.set_valve(retract, True)
        # Wait time for piston to fully retract
        await self.nh.sleep(self._actuator_timing["retract_wait_time"])
        # Lock thruster in place
        await self.set_valve(unlock, False)
        # Wait some time for lock to engage
        await self.nh.sleep(self._actuator_timing["retract_lock_time"])
        # Stop pulling up
        await self.set_valve(retract, False)

    async def deploy_thrusters(self) -> None:
        """
        Deploy all 4 thrusters simultaneously.
        TODO: perform in sequence after testing has been done to see if this is needed
        """
        await asyncio.gather(
            self.deploy_thruster(name) for name in ["FL", "FR", "BL", "BR"]
        )

    async def retract_thrusters(self) -> None:
        """
        Retract all 4 thrusters simultaneously.
        TODO: perform in sequence after testing has been done to see if this is needed
        """
        await asyncio.gather(
            self.retract_thruster(name) for name in ["FL", "FR", "BL", "BR"]
        )

    async def reload_launcher(self):
        if self.launcher_state != "inactive":
            raise Exception(f"Launcher is {self.launcher_state}")
        self.launcher_state = "reloading"
        await self.set_valve("LAUNCHER_RELOAD_EXTEND", True)
        await self.set_valve("LAUNCHER_RELOAD_RETRACT", False)
        await self.nh.sleep(self._actuator_timing["launcher_reload_extend_time"])
        await self.set_valve("LAUNCHER_RELOAD_EXTEND", False)
        await self.set_valve("LAUNCHER_RELOAD_RETRACT", True)
        await self.nh.sleep(self._actuator_timing["launcher_reload_retract_time"])
        await self.set_valve("LAUNCHER_RELOAD_EXTEND", False)
        await self.set_valve("LAUNCHER_RELOAD_RETRACT", False)
        self.launcher_state = "inactive"

    async def fire_launcher(self):
        if self.launcher_state != "inactive":
            raise Exception(f"Launcher is {self.launcher_state}")
        self.launcher_state = "firing"
        await self.set_valve("LAUNCHER_FIRE", True)
        await self.nh.sleep(0.5)
        self.launcher_state = "inactive"

    def set_valve(self, name, state):
        req = SetValveRequest(actuator=name, opened=state)
        return self._actuator_client(req)

    async def get_sorted_objects(
        self,
        name: str,
        n: int = -1,
        throw: bool = True,
        **kwargs,
    ):
        """
        Get the closest N objects with a particular name from the PCODAR database
        @param name: the name of the object
        @param n: the number of objects to get, if -1, get all of them
        @param throw: If true, raise exception if not enough objects present
        @param **kwargs: other kwargs to give to database_query
        @return tuple([sorted_object_messages, [object_positions]) of sorted object messages
                and their positions as a Nx3 numpy array.
        """
        objects = (await self.database_query(object_name=name, **kwargs)).objects
        if n != -1 and len(objects) < n:
            if throw:
                raise Exception(f"Could not get {n} {name} objects")
            else:
                n = len(objects)
        if n == 0:
            return None
        positions = np.empty((len(objects), 3))
        for i, obj in enumerate(objects):
            positions[i, :] = mil_tools.rosmsg_to_numpy(obj.pose.position)
        nav_pose = (await self.tx_pose())[0]
        distances = np.linalg.norm(positions - nav_pose, axis=1)
        distances_argsort = np.argsort(distances)
        if n != -1:
            distances_argsort = distances_argsort[:n]
        objects_sorted = [objects[i] for i in distances_argsort]
        return (objects_sorted, positions[distances_argsort, :])

    async def database_query(
        self,
        object_name: str | None = None,
        raise_exception: bool = True,
        **kwargs,
    ):
        if object_name is not None:
            kwargs["name"] = object_name
            res = await self._database_query(ObjectDBQueryRequest(**kwargs))

            if not res.found and raise_exception:
                raise MissingPerceptionObject(kwargs["name"])

            return res

        res = await self._database_query(ObjectDBQueryRequest(**kwargs))
        return res

    async def camera_database_query(self, object_name: str | None = None, **kwargs):
        if object_name is not None:
            kwargs["name"] = object_name
            res = await self._camera_database_query(
                navigator_srvs.CameraDBQueryRequest(**kwargs),
            )

            return res

        res = await self._database_query(navigator_srvs.CameraDBQueryRequest(**kwargs))
        return res

    def vision_request(self, request_name, **kwargs):
        fprint("DEPRECATED: Please use new dictionary based system.")
        return self.vision_proxies[request_name].get_response(**kwargs)

    @classmethod
    def change_wrench(cls, source):
        return cls._change_wrench(MuxSelectRequest(source))

    @classmethod
    def disable_thrusters(cls):
        """
        Disable thrusters by setting the wrench source to none
        """
        return cls.change_wrench("__none")

    @classmethod
    def enable_autonomous(cls):
        """
        Enable autonmous wrench, useful to call after disable_thrusters to regain control
        """
        return cls.change_wrench("autonomous")

    @classmethod
    async def change_trajectory(cls, source: str):
        return await cls._change_trajectory(MuxSelectRequest(source))

    def search(self, *args, **kwargs):
        return Searcher(self, *args, **kwargs)

    async def latching_publisher(
        self,
        topic: str,
        msg_type: type[genpy.Message],
        msg: genpy.Message,
        freq: int = 2,
        update_header: bool = True,
    ):
        """
        Creates a publisher that publishes a message at a set frequency.
        Usage:
            latched = navigator.latching_publisher("/ogrid", OccupancyGrid, my_ogrid)
            # Now the ogrid is publishing twice a second...
            # If you want to cancel the publisher:
            latched.cancel()
        """
        pub = self.nh.advertise(topic, msg_type)
        await pub.setup()

        while True:
            # Update timestamps if the msg has a header
            if update_header and hasattr(msg, "header"):
                msg.header.stamp = self.nh.get_time()

            pub.publish(msg)
            await self.nh.sleep(1.0 / freq)

    @classmethod
    def _load_vision_services(cls, fname: str = "vision_services.yaml"):
        rospack = rospkg.RosPack()
        config_file = os.path.join(
            rospack.get_path("navigator_missions"),
            "launch",
            fname,
        )
        f = yaml.safe_load(open(config_file))

        for name in f:
            try:
                s_type = getattr(navigator_srvs, f[name]["type"])
                s_req = getattr(navigator_srvs, "{}Request".format(f[name]["type"]))
                s_args = f[name]["args"]
                s_client = cls.nh.get_service_client(f[name]["topic"], s_type)
                s_switch = cls.nh.get_service_client(
                    f[name]["topic"] + "/switch",
                    SetBool,
                )
                cls.vision_proxies[name] = VisionProxy(
                    s_client,
                    s_req,
                    s_args,
                    s_switch,
                )
            except Exception as e:
                err = f"Error loading vision services: {e}"
                fprint("" + err, title="NAVIGATOR", msg_color="red")

    @classmethod
    def _load_mission_params(cls, fname="mission_params.yaml"):
        rospack = rospkg.RosPack()
        config_file = os.path.join(
            rospack.get_path("navigator_missions"),
            "launch",
            fname,
        )
        f = yaml.safe_load(open(config_file))

        for name in f:
            try:
                param = f[name]["param"]
                options = f[name]["options"]
                desc = f[name]["description"]
                default = f[name].get("default")
                cls.mission_params[name] = MissionParam(
                    cls.nh,
                    param,
                    options,
                    desc,
                    default,
                )
            except Exception as e:
                err = f"Error loading mission params: {e}"
                fprint("" + err, title="NAVIGATOR", msg_color="red")

    @classmethod
    def kill_alarm_cb(cls, _, alarm):
        cls.killed = alarm.raised
        cls.kill_alarm = alarm

    @classmethod
    async def _make_alarms(cls):
        cls.kill_listener = await TxAlarmListener.init(
            cls.nh,
            "kill",
            cls.kill_alarm_cb,
        )
        await cls.kill_listener.setup()
        # TODO: Enable node handle subscriber to topic to have multiple callbacks
        # cls.odom_loss_listener = await TxAlarmListener.init(
        #     cls.nh,
        #     "odom-kill",
        #     lambda _, alarm: setattr(cls, "odom_loss", alarm.raised),
        # )
        fprint("Alarm listener created, listening to alarms: ", title="NAVIGATOR")

        cls.kill_alarm = await cls.kill_listener.get_alarm()
        cls.killed = cls.kill_alarm.raised
        cls.odom_loss = False  # await cls.odom_loss_listener.is_raised()
        fprint("\tkill :", newline=False)
        fprint(cls.killed)
        fprint("\todom-kill :", newline=False)
        fprint(cls.odom_loss)


class VisionProxy:
    def __init__(
        self,
        client: ServiceClient,
        request: ServiceClient,
        args,
        switch: Callable[[SetBoolRequest], SetBoolResponse],
    ):
        self.client = client
        self.request = request
        self.args = args
        self.switch = switch

    def start(self):
        return self.switch(SetBoolRequest(data=True))

    def stop(self):
        return self.switch(SetBoolRequest(data=False))

    def get_response(self, **kwargs):
        s_args = dict(self.args.items() + kwargs.items())
        s_req = self.request(**s_args)

        return self.client(s_req)


class MissionParam:
    def __init__(self, nh: NodeHandle, param, options, desc, default):
        self.nh = nh
        self.param = param
        self.options = options
        self.description = desc
        self.default = default

    async def get(self, raise_exception: bool = True):
        if not (await self.exists()):
            if self.default is not None:
                await self.set(self.default)
                return self.default
            if raise_exception:
                raise Exception(f"Mission Param {self.param} not yet set")
            else:
                return False
        value = await self.nh.get_param(self.param)
        if not self._valid(value):
            raise Exception(
                "Value {} is invalid for param {}\nValid values: {}\nDescription: {}".format(
                    value,
                    self.param,
                    self.options,
                    self.description,
                ),
            )
        else:
            return value

    async def exists(self) -> bool:
        return await self.nh.has_param(self.param)

    async def set(self, value):
        if not self._valid(value):
            raise Exception(
                "Value {} is invalid for param {}\nValid values: {}\nDescription: {}".format(
                    value,
                    self.param,
                    self.options,
                    self.description,
                ),
            )
        await self.nh.set_param(self.param, value)

    async def valid(self):
        exists = await self.exists()
        if not exists:
            return False
        value = await self.nh.get_param(self.param)
        if not self._valid(value):
            return False
        return True

    async def reset(self):
        if await self.exists():
            if self.default is not None:
                await self.set(self.default)
            else:
                await self.nh.delete_param(self.param)

    def _valid(self, value) -> bool:
        return any(x == value for x in self.options)


class Searcher:
    def __init__(
        self,
        nav: NaviGatorMission,
        search_pattern=None,
        looker=None,
        vision_proxy: str = "test",
        **kwargs,
    ):
        self.nav = nav
        self.looker = looker
        if looker is None:
            self.looker = asyncio.create_task(self._vision_proxy_look)
            self.vision_proxy = vision_proxy
        self.looker_kwargs = kwargs
        self.search_pattern = search_pattern

        self.object_found = False
        self.pattern_done = False
        self.response = None
        print("dfdf")

    async def start_search(self, timeout=120, loop=True, spotings_req=2, **kwargs):
        fprint("Starting.", title="SEARCHER")
        looker = asyncio.create_task(self._run_look(spotings_req))
        finder = asyncio.create_task(self._run_search_pattern(loop, **kwargs))

        start_pose = self.nav.move.forward(0)
        start_time = self.nav.nh.get_time()
        try:
            while self.nav.nh.get_time() - start_time < genpy.Duration(timeout):
                if self.object_found:
                    finder.cancel()
                    fprint("Object found.", title="SEARCHER")
                    return True
                if self.pattern_done and not loop:
                    finder.cancel()
                    return False
                await self.nav.nh.sleep(0.1)

        except KeyboardInterrupt:
            # This doesn't work...
            fprint("Control C detected!", title="SEARCHER")

        fprint("Object NOT found. Returning to start position.", title="SEARCHER")
        finder.cancel()
        looker.cancel()

        start_pose.go()

    async def _run_search_pattern(self, loop, **kwargs):
        """
        Look around using the search pattern.
        If `loop` is true, then keep iterating over the list until timeout is reached or we find it.
        """
        if self.search_pattern is None:
            return

        async def pattern():
            for pose in self.search_pattern:
                fprint("Going to next position.", title="SEARCHER")
                if isinstance(pose, (list, np.ndarray)):
                    await self.nav.move.relative(pose).go(**kwargs)
                else:
                    await pose.go(**kwargs)

                await self.nav.nh.sleep(2)
            if not loop:
                fprint("Search Pattern Over", title="SEARCHER")
                self.pattern_done = True

        fprint("Executing search pattern.", title="SEARCHER")

        if loop:
            while True:
                await pattern()
        else:
            await pattern()

    async def _vision_proxy_look(self):
        resp = await self.nav.vision_proxies[self.vision_proxy].get_response(
            **self.looker_kwargs,
        )
        return resp.found

    async def _run_look(self, spotings_req):
        """
        Look for the object using the vision proxy.
        Only return true when we spotted the objects `spotings_req` many times (for false positives).
        """
        spotings = 0
        fprint("Looking for object.", title="SEARCHER")
        while spotings < spotings_req:
            if await self.looker(**self.looker_kwargs):
                fprint(
                    f"Object found! {spotings + 1}/{spotings_req}",
                    title="SEARCHER",
                )
                spotings += 1
            else:
                spotings = 0

            await self.nav.nh.sleep(0.5)

        if spotings >= spotings_req:
            self.object_found = True


async def main():
    nh = NodeHandle.from_argv("navigator_singleton")
    await nh.setup()
    n = await NaviGatorMission(nh).setup_base()
    fprint(await n.vision_proxies["start_gate"].get_response())


if __name__ == "__main__":
    uvloop.install()
    asyncio.run(main())
