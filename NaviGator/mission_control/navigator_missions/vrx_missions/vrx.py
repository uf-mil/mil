#!/usr/bin/env python
from __future__ import division
import txros
import numpy as np
from twisted.internet import defer
from robot_localization.srv import FromLL, FromLLRequest, ToLL, ToLLRequest
from vrx_gazebo.msg import Task
from vrx_gazebo.srv import ColorSequence
from geographic_msgs.msg import GeoPoseStamped, GeoPath
from std_msgs.msg import Float64, Float64MultiArray, String
from navigator_missions import Navigator
from mil_tools import rosmsg_to_numpy, numpy_to_point
from sensor_msgs.msg import Image, CameraInfo

___author___ = "Kevin Allen"


class Vrx(Navigator):
    def __init__(self, *args, **kwargs):
        super(Vrx, self).__init__(*args, **kwargs)

    @staticmethod
    def init():
        if hasattr(Vrx, '_vrx_init'):
            return
        print 'cALLING INITITTI'
        Vrx.from_lla = Vrx.nh.get_service_client("/fromLL", FromLL)
        Vrx.to_lla = Vrx.nh.get_service_client("/toLL", ToLL)
        Vrx.task_info_sub = Vrx.nh.subscribe("/vrx/task/info", Task)
        Vrx.scan_dock_color_sequence = Vrx.nh.get_service_client("/vrx/scan_dock/color_sequence", ColorSequence)
        Vrx.station_keep_goal = Vrx.nh.subscribe("/vrx/station_keeping/goal", GeoPoseStamped)
        Vrx.wayfinding_path_sub = Vrx.nh.subscribe("/vrx/wayfinding/waypoints", GeoPath)
        Vrx.station_keeping_pose_error = Vrx.nh.subscribe("/vrx/station_keeping/pose_error", Float64)
        Vrx.station_keeping_rms_error = Vrx.nh.subscribe("/vrx/station_keeping/rms_error", Float64)
        Vrx.wayfinding_min_errors = Vrx.nh.subscribe("/vrx/wayfinding/min_errors", Float64MultiArray)
        Vrx.wayfinding_mean_error = Vrx.nh.subscribe("/vrx/wayfinding/mean_error", Float64)
        Vrx.perception_landmark = Vrx.nh.advertise("/vrx/perception/landmark", GeoPoseStamped)
        #Vrx.scan_dock_placard_symbol = Vrx.nh.subscribe("/vrx/scan_dock/placard_symbol", String)

        Vrx.front_left_camera_info_sub = None 
        Vrx.front_left_camera_sub = None


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
    def get_closest(self):
        ret = yield self.get_sorted_objects('all')

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        print 'VRXXX'
        yield self.set_vrx_classifier_enabled.wait_for_service()
        print 'ggf'
        yield self._pcodar_set_params.wait_for_service()
        print 'gfgf'
        msg = yield self.task_info_sub.get_next_message()
        task_name = msg.name
        print 'GET TASK'
        if task_name == 'stationkeeping':
            yield self.run_submission('VrxStationKeeping')
        elif task_name == 'wayfinding':
            yield self.run_submission('VrxWayfinding')
        elif task_name == 'navigation_course':
            yield self.run_submission('VrxNavigation')
        elif task_name == 'perception':
            yield self.run_submission('VrxPerception')
        elif task_name == 'scan_and_dock':
            yield self.run_submission('ScanAndDock')
        elif task_name == 'scan':
            yield self.run_submission('DockDriver')
        defer.returnValue(msg)
