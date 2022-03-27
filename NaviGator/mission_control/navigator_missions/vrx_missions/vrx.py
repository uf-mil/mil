#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
import math
from twisted.internet import defer
from robot_localization.srv import FromLL, FromLLRequest, ToLL, ToLLRequest
from navigator_msgs.srv import AcousticBeacon, ChooseAnimal, MoveToWaypoint, MoveToWaypointRequest
from vrx_gazebo.msg import Task
from vrx_gazebo.srv import ColorSequence
from geographic_msgs.msg import GeoPoseStamped, GeoPath
from darknet_ros_msgs.msg import BoundingBoxes
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray, String, Int32, Empty
from navigator_missions import Navigator
from mil_tools import rosmsg_to_numpy, numpy_to_point
from sensor_msgs.msg import Image, CameraInfo
from txros import action, util, tf, NodeHandle
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from navigator_msgs.srv import AverageScale

___author___ = "Kevin Allen"


class Vrx(Navigator):
    def __init__(self, *args, **kwargs):
        super(Vrx, self).__init__(*args, **kwargs)

    @staticmethod
    def init():
        if hasattr(Vrx, '_vrx_init'):
            return
        Vrx.from_lla = Vrx.nh.get_service_client("/fromLL", FromLL)
        Vrx.to_lla = Vrx.nh.get_service_client("/toLL", ToLL)
        Vrx.task_info_sub = Vrx.nh.subscribe("/vrx/task/info", Task)
        Vrx.scan_dock_color_sequence = Vrx.nh.get_service_client("/vrx/scan_dock_deliver/color_sequence", ColorSequence)
        Vrx.fire_ball = Vrx.nh.advertise("/wamv/shooters/ball_shooter/fire", Empty)
        Vrx.station_keep_goal = Vrx.nh.subscribe("/vrx/station_keeping/goal", GeoPoseStamped)
        Vrx.wayfinding_path_sub = Vrx.nh.subscribe("/vrx/wayfinding/waypoints", GeoPath)
        Vrx.station_keeping_pose_error = Vrx.nh.subscribe("/vrx/station_keeping/pose_error", Float64)
        Vrx.station_keeping_rms_error = Vrx.nh.subscribe("/vrx/station_keeping/rms_error", Float64)
        Vrx.wayfinding_min_errors = Vrx.nh.subscribe("/vrx/wayfinding/min_errors", Float64MultiArray)
        Vrx.wayfinding_mean_error = Vrx.nh.subscribe("/vrx/wayfinding/mean_error", Float64)
        Vrx.perception_landmark = Vrx.nh.advertise("/vrx/perception/landmark", GeoPoseStamped)

        Vrx.animal_landmarks = Vrx.nh.subscribe("/vrx/wildlife/animals/poses", GeoPath)
        Vrx.beacon_landmark = Vrx.nh.get_service_client("beaconLocator", AcousticBeacon)
        Vrx.circle_animal = Vrx.nh.get_service_client("/choose_animal", ChooseAnimal)
        Vrx.set_long_waypoint = Vrx.nh.get_service_client("/set_long_waypoint", MoveToWaypoint)
        Vrx.darknet_objects = Vrx.nh.subscribe("/darknet_ros/bounding_boxes", BoundingBoxes)
        Vrx.tf_listener = tf.TransformListener(Vrx.nh)
        Vrx.database_response = Vrx.nh.get_service_client('/database/requests', ObjectDBQuery)
        Vrx.pcodar_scales = Vrx.nh.get_service_client('/pcodar_average_scale_service', AverageScale)
        #Vrx.scan_dock_placard_symbol = Vrx.nh.subscribe("/vrx/scan_dock/placard_symbol", String)

        Vrx.front_left_camera_info_sub = None 
        Vrx.front_left_camera_sub = None
        Vrx.front_right_camera_info_sub = None 
        Vrx.front_right_camera_sub = None


        Vrx._vrx_init = True

    #@txros.util.cancellableInlineCallbacks
    def cleanup(self):
        pass

    @staticmethod
    def init_front_left_camera():
        if Vrx.front_left_camera_sub is None:
            Vrx.front_left_camera_sub = Vrx.nh.subscribe(
                "/wamv/sensors/cameras/front_left_camera/image_raw", Image)

        if Vrx.front_left_camera_info_sub is None:
            Vrx.front_left_camera_info_sub = Vrx.nh.subscribe(
                "/wamv/sensors/cameras/front_left_camera/camera_info", CameraInfo)

    @staticmethod
    def init_front_right_camera():
        if Vrx.front_right_camera_sub is None:
            Vrx.front_right_camera_sub = Vrx.nh.subscribe(
                "/wamv/sensors/cameras/front_right_camera/image_raw", Image)

        if Vrx.front_right_camera_info_sub is None:
            Vrx.front_right_camera_info_sub = Vrx.nh.subscribe(
                "/wamv/sensors/cameras/front_right_camera/camera_info", CameraInfo)


    @txros.util.cancellableInlineCallbacks
    def geo_pose_to_enu_pose(self, geo):
        self.send_feedback('Waiting for LLA conversion')
        enu_msg = yield self.from_lla(FromLLRequest(ll_point=geo.position))
        position_enu = rosmsg_to_numpy(enu_msg.map_point)
        orientation_enu = rosmsg_to_numpy(geo.orientation)
        defer.returnValue((position_enu, orientation_enu))

    @txros.util.cancellableInlineCallbacks
    def enu_position_to_geo_point(self, enu_array):
        self.send_feedback('Waiting for LLA conversion')
        lla_msg = yield self.to_lla(ToLLRequest(map_point=numpy_to_point(enu_array)))
        defer.returnValue(lla_msg.ll_point)

    @txros.util.cancellableInlineCallbacks
    def get_latching_msg(self, sub):
        msg = yield sub.get_last_message()
        if msg is None:
            msg = yield sub.get_next_message()
        defer.returnValue(msg)

    @txros.util.cancellableInlineCallbacks
    def wait_for_task_such_that(self, f):
        while True:
            msg = yield self.task_info_sub.get_next_message()
            if f(msg):
                defer.returnValue(None)

    @txros.util.cancellableInlineCallbacks
    def point_at_goal(self, goal_pos):
        vect = [ goal_pos[0] - self.pose[0][0], goal_pos[1] - self.pose[0][1]]
        theta = math.atan2(vect[1], vect[0])
        orientation_fix = tf.transformations.quaternion_from_euler(0,0,theta)
        yield self.move.set_orientation(orientation_fix).go(blind=True)

    @txros.util.cancellableInlineCallbacks
    def send_trajectory_without_path(self, goal_pose):
        req = MoveToWaypointRequest()
        req.target_p.position.x = goal_pose[0][0]
        req.target_p.position.y = goal_pose[0][1]
        req.target_p.position.z = goal_pose[0][2]
        req.target_p.orientation.x = goal_pose[1][0]
        req.target_p.orientation.y = goal_pose[1][1]
        req.target_p.orientation.z = goal_pose[1][2]
        req.target_p.orientation.w = goal_pose[1][3]
        yield self.set_long_waypoint(req)

    @txros.util.cancellableInlineCallbacks
    def get_closest(self):
        ret = yield self.get_sorted_objects('all')

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        yield self.set_vrx_classifier_enabled.wait_for_service()
        yield self._pcodar_set_params.wait_for_service()
        msg = yield self.task_info_sub.get_next_message()
        task_name = msg.name
        if task_name == 'station_keeping':
            yield self.run_submission('VrxStationKeeping2')
        elif task_name == 'wayfinding':
            yield self.run_submission('VrxWayfinding2')
        elif task_name == 'gymkhana':
            yield self.run_submission('Gymkhana')
        elif task_name == 'perception':
            yield self.run_submission('VrxPerception')
        elif task_name == 'wildlife':
            yield self.run_submission('VrxWildlife')
        elif task_name == 'scan_dock_deliver':
            yield self.run_submission('ScanAndDock')
        msg = yield self.task_info_sub.get_next_message()
        defer.returnValue(msg)
