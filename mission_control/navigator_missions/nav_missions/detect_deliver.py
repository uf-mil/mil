#!/usr/bin/env python
from __future__ import division
import txros
import std_srvs.srv
import numpy as np
import tf
import tf.transformations as trns
from navigator_msgs.msg import ShooterDoAction, ShooterDoActionGoal
from navigator_msgs.srv import CameraToLidarTransform,CameraToLidarTransformRequest
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool, SetBoolRequest
from twisted.internet import defer
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker,MarkerArray
import navigator_tools
from navigator_tools import fprint, MissingPerceptionObject

class DetectDeliverMission:
    # Note, this will be changed when the shooter switches to actionlib
    shoot_distance_meters = 2.7
    theta_offset = np.pi / 2.0
    spotings_req = 1
    circle_radius = 10
    search_timeout_seconds = 300
    SHAPE_CENTER_TO_BIG_TARGET = 0.42
    SHAPE_CENTER_TO_SMALL_TARGET = -0.42
    NUM_BALLS = 4

    def __init__(self, navigator):
        self.navigator = navigator
        self.cameraLidarTransformer = navigator.nh.get_service_client("/camera_to_lidar/right_right_cam", CameraToLidarTransform)
        self.shooterLoad = txros.action.ActionClient(
            self.navigator.nh, '/shooter/load', ShooterDoAction)
        self.shooterFire = txros.action.ActionClient(
            self.navigator.nh, '/shooter/fire', ShooterDoAction)
        self.identified_shapes = []
        self.last_shape_error = ""
        self.last_lidar_error = ""
        self.found_shape = None
        self.normal_res = None

    def _bounding_rect(self,points):
        np_points = map(navigator_tools.point_to_numpy, points)
        xy_max = np.max(np_points, axis=0)
        xy_min = np.min(np_points, axis=0)
        return np.append(xy_max, xy_min)

    @txros.util.cancellableInlineCallbacks
    def set_shape_and_color(self):
        target = yield self.navigator.mission_params["detect_deliver_target"].get()
        if target == "BIG":
            self.target_offset_meters = self.SHAPE_CENTER_TO_BIG_TARGET
        elif target == "SMALL":
            self.target_offset_meters = self.SHAPE_CENTER_TO_SMALL_TARGET
        self.Shape = yield self.navigator.mission_params["detect_deliver_shape"].get()
        self.Color = yield self.navigator.mission_params["detect_deliver_color"].get()
        fprint("Color={} Shape={} Target={}".format(self.Color, self.Shape, target), title="DETECT DELIVER",  msg_color='green')

    @txros.util.cancellableInlineCallbacks
    def get_waypoint(self):
        res = yield self.navigator.database_query("shooter")
        if not res.found:
            fprint("shooter waypoint not found", title="DETECT DELIVER",  msg_color='red')
            raise MissingPerceptionObject("shooter", "Detect Deliver Waypoint not found")
        self.waypoint_res = res

    @txros.util.cancellableInlineCallbacks
    def circle_search(self):
        print "Starting circle search"
        #yield self.navigator.move.look_at(navigator_tools.rosmsg_to_numpy(self.waypoint_res.objects[0].position)).go()
        pattern = self.navigator.move.d_circle_point(navigator_tools.rosmsg_to_numpy(
            self.waypoint_res.objects[0].position), radius=self.circle_radius, theta_offset=self.theta_offset, direction='cw')
        yield next(pattern).go()
        searcher = self.navigator.search(search_pattern=pattern, looker=self.search_shape)
        yield searcher.start_search(timeout=self.search_timeout_seconds, spotings_req=self.spotings_req, move_type="skid", loop=False)
        print "Ended Circle Search"

        #  fprint("Starting Circle Search", title="DETECT DELIVER",  msg_color='green')
        #  yield self.navigator.move.look_at(navigator_tools.rosmsg_to_numpy(self.waypoint_res.objects[0].position)).yaw_left(90, unit='deg').go()
        #  self.navigator.move.d_circle_point(navigator_tools.rosmsg_to_numpy(self.waypoint_res.objects[0].position), self.circle_radius).go()
        #  while not (yield self.is_found()):
            #  fprint("Shape not found Error={}".format(self.resp.error), title="DETECT DELIVER", msg_color='red')
        #  yield self.navigator.move.stop()
        #  fprint("Ended Circle Search", title="DETECT DELIVER",  msg_color='green')

    def update_shape(self, shape_res, normal_res, tf):
       for index, (shape, normal, enu_cam) in enumerate(self.identified_shapes):
            if shape_res.Color == shape.Color and shape_res.Shape == shape.Shape:
                self.identified_shapes[index] = (shape_res, normal_res, tf)
                return
       self.identified_shapes.append((shape_res, normal_res, tf))

    def correct_shape(self, shape):
        return (self.Color == "ANY" or self.Color == shape.Color) and (self.Shape == "ANY" or self.Shape == shape.Shape)

    @txros.util.cancellableInlineCallbacks
    def search_shape(self):
        shapes = yield self.get_shape()
        if shapes.found:
            for shape in shapes.shapes.list:
                normal_res = yield self.get_normal(shape)
                if normal_res.success:
                    enu_cam_tf = yield self.navigator.tf_listener.get_transform('/enu', '/'+shape.header.frame_id, shape.header.stamp)
                    self.update_shape(shape, normal_res, enu_cam_tf)
                    if self.correct_shape(shape):
                        self.found_shape = shape
                        self.normal_res = normal_res
                        self.enu_cam_tf = enu_cam_tf
                        defer.returnValue(True)
                else:
                    if not self.last_lidar_error == normal_res.error:
                        fprint("Normal notfound Error={}".format(normal_res.error), title="DETECT DELIVER", msg_color='red')
                    self.last_lidar_error = normal_res.error
        else:
            if not self.last_shape_error == shapes.error:
                fprint("shape not found Error={}".format(shapes.error), title="DETECT DELIVER", msg_color="red")
            self.last_shape_error = shapes.error
        defer.returnValue(False)

    def select_backup_shape(self):
        if len(self.identified_shapes) == 0:
            raise Exception("No Shapes seen")
        for cur in self.identified_shapes:
            if cur[0].Shape == self.Shape:
                return cur
            if cur[0].Color == self.Color:
                return cur
        return self.identified_shapes[0]

    @txros.util.cancellableInlineCallbacks
    def align_to_target(self):
        if self.found_shape == None or self.normal_res == None:
            shape, normal_res, tf = self.select_backup_shape()
            self.normal_res = normal_res
            self.found_shape = shape
            self.enu_cam_tf = tf
        shooter_baselink_tf = yield self.navigator.tf_listener.get_transform('/base_link','/shooter')
        shape_point, shape_norm = yield self.get_shape_pos(self.normal_res, self.found_shape.header.frame_id)
        goal_point, goal_orientation = self.get_aligned_pose(shape_point, shape_norm)
        move = self.navigator.move.set_position(goal_point).set_orientation(goal_orientation).forward(self.target_offset_meters)
        move = move.left(-shooter_baselink_tf._p[1]).forward(-shooter_baselink_tf._p[0]) #Adjust for location of shooter
        fprint("Aligning to shoot at {}".format(move), title="DETECT DELIVER", msg_color='green')
        yield move.go(move_type="skid")

    def get_shape(self):
        return self.navigator.vision_proxies["get_shape"].get_response(Shape="ANY", Color="ANY")

    def get_aligned_pose(self, enupoint, enunormal):
        aligned_position = enupoint + self.shoot_distance_meters * enunormal  # moves x meters away
        angle = np.arctan2(-enunormal[0], enunormal[1])
        aligned_orientation = trns.quaternion_from_euler(0, 0, angle)  # Align perpindicular
        return (aligned_position, aligned_orientation)

    def get_shape_pos(self, normal_res, frame):
        enunormal = self.enu_cam_tf.transform_vector(navigator_tools.rosmsg_to_numpy(normal_res.normal))
        enupoint = self.enu_cam_tf.transform_point(navigator_tools.rosmsg_to_numpy(normal_res.closest))
        return (enupoint, enunormal)

    @txros.util.cancellableInlineCallbacks
    def get_normal(self, shape):
        req = CameraToLidarTransformRequest()
        req.header = shape.header
        req.point = Point()
        req.point.x = shape.CenterX
        req.point.y = shape.CenterY
        rect = self._bounding_rect(shape.points)
        req.tolerance = int(min(rect[0]-rect[3],rect[1]-rect[4])/2.0)
        normal_res = yield self.cameraLidarTransformer(req)
        if not self.normal_is_sane(normal_res.normal):
            normal_res.success = False
            normal_res.error = "UNREASONABLE NORMAL"
        defer.returnValue(normal_res)

    def normal_is_sane(self, vector3):
         return abs(navigator_tools.rosmsg_to_numpy(vector3)[1]) < 0.2

    @txros.util.cancellableInlineCallbacks
    def shoot_all_balls(self):
        for i in range(self.NUM_BALLS):
            goal = yield self.shooterLoad.send_goal(ShooterDoAction())
            fprint("Loading Shooter {}".format(i), title="DETECT DELIVER",  msg_color='green')
            res = yield goal.get_result()
            yield self.navigator.nh.sleep(2)
            goal = yield self.shooterFire.send_goal(ShooterDoAction())
            fprint("Firing Shooter {}".format(i), title="DETECT DELIVER",  msg_color='green')
            res = yield goal.get_result()

    @txros.util.cancellableInlineCallbacks
    def find_and_shoot(self):
        yield self.navigator.vision_proxies["get_shape"].start()
        yield self.set_shape_and_color()  # Get correct goal shape/color from params
        yield self.get_waypoint()  # Get waypoint of shooter target
        yield self.circle_search()  # Go to waypoint and circle until target found
        yield self.align_to_target()
        yield self.shoot_all_balls()
        yield self.navigator.vision_proxies["get_shape"].stop()

@txros.util.cancellableInlineCallbacks
def setup_mission(navigator):
    color = "ANY"
    shape = "ANY"
    #color = yield navigator.mission_params["scan_the_code_color3"].get()
    fprint("Setting search shape={} color={}".format(shape, color), title="DETECT DELIVER",  msg_color='green')
    yield navigator.mission_params["detect_deliver_shape"].set(shape)
    yield navigator.mission_params["detect_deliver_color"].set(color)

@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    yield setup_mission(navigator)
    fprint("STARTING MISSION", title="DETECT DELIVER",  msg_color='green')
    mission = DetectDeliverMission(navigator)
    yield mission.find_and_shoot()
    fprint("ENDING MISSION", title="DETECT DELIVER",  msg_color='green')
