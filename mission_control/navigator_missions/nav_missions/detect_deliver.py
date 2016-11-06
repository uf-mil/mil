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


class DetectDeliverMission:
    # Note, this will be changed when the shooter switches to actionlib
    shoot_distance_meters = 7
    theta_offset = np.pi / 2.0
    spotings_req = 1
    circle_radius = 7
    search_timeout_seconds = 300
    target_offset_meters = 0
    normal_approx_tolerance_proportion = 0.035

    def __init__(self, navigator):
        self.navigator = navigator
        self.cameraLidarTransformer = navigator.nh.get_service_client("/camera_to_lidar/right_right_cam", CameraToLidarTransform)
        self.shooterLoad = txros.action.ActionClient(
            self.navigator.nh, '/shooter/load', ShooterDoAction)
        self.shooterFire = txros.action.ActionClient(
            self.navigator.nh, '/shooter/fire', ShooterDoAction)
        self.markers_pub = self.navigator.nh.advertise("/detect_deliver/debug_marker",MarkerArray)

    @txros.util.cancellableInlineCallbacks
    def set_shape_and_color(self):
        self.Shape = yield self.navigator.mission_params["detect_deliver_shape"].get()
        self.Color = yield self.navigator.mission_params["detect_deliver_color"].get()
        print "Looking for ", self.Color, " ", self.Shape

    @txros.util.cancellableInlineCallbacks
    def get_waypoint(self):
        res = yield self.navigator.database_query("shooter")
        if not res.found:
            print "Waypoint not found in database, exiting"
            raise Exception('Waypoint not found')
        self.waypoint_res = res

    @txros.util.cancellableInlineCallbacks
    def circle_search(self):
        print "Starting circle search"
        #yield self.navigator.move.look_at(navigator_tools.rosmsg_to_numpy(self.waypoint_res.objects[0].position)).go()
        pattern = self.navigator.move.circle_point(navigator_tools.rosmsg_to_numpy(
            self.waypoint_res.objects[0].position), radius=self.circle_radius,  theta_offset=self.theta_offset)
        yield next(pattern).go()
        searcher = self.navigator.search(
            vision_proxy='get_shape', search_pattern=pattern, Shape=self.Shape, Color=self.Color)
        yield searcher.start_search(timeout=self.search_timeout_seconds, spotings_req=self.spotings_req, move_type="skid")
        print "Ended Circle Search"

    @txros.util.cancellableInlineCallbacks
    def align_to_target(self):
        self.markers = MarkerArray()
        while not (yield self.is_found()):
          print "Shape Not Found, Error: ", self.resp.error
        while not (yield self.get_normal()):
          print "Normal Not Found, Error: ", self.normal_res.error
        yield self.get_aligned_pos()
        yield self.markers_pub.publish(self.markers)   
        print "Aligning to Position: ", self.aligned_position
        print "Aligning to Orientation: ", self.aligned_orientation
        yield self.navigator.move.set_position(self.aligned_position).set_orientation(self.aligned_orientation).go(move_type="skid")

    @txros.util.cancellableInlineCallbacks
    def offset_for_target(self):   
        # Move off of center of shape to be centered with large target
        move = self.navigator.move.forward(self.target_offset_meters)
        marker = Marker()
        marker.scale.x = 1;
        marker.scale.y = .1;
        marker.scale.z = .1;
        marker.action = Marker.ADD;
        marker.header.frame_id = "enu"
        marker.header.stamp = self.navigator.nh.get_time()
        marker.pose = self.navigator.move.goal.goal
        marker.type = Marker.ARROW
        marker.text = "Offset goal"
        marker.ns = "detect_deliver"
        marker.id = 3
        marker.color.b = 1
        marker.color.a = 1
        self.markers.markers.append(marker)
        yield move

    @txros.util.cancellableInlineCallbacks
    def is_found(self):
        self.resp = yield self.navigator.vision_proxies["get_shape"].get_response(Shape=self.Shape, Color=self.Color)
        #  print "Resp ",self.resp
        if self.resp.found:
            self.found_shape = self.resp.shapes.list[0]
        defer.returnValue(self.resp.found)

    def get_aligned_pos(self):
      # ~position, rot = yield self.navigator.tx_pose
      
      self.aligned_position = self.enupoint + self.shoot_distance_meters * self.enunormal  # moves x meters away
      angle = np.arctan2(-self.enunormal[0], self.enunormal[1])
      self.aligned_orientation = trns.quaternion_from_euler(0, 0, angle)  # Align perpindicular

      move_marker = Marker()
      move_marker.scale.x = 1;
      move_marker.scale.y = .1;
      move_marker.scale.z = .1;
      move_marker.action = Marker.ADD;
      move_marker.header.frame_id = "enu"
      move_marker.header.stamp = self.navigator.nh.get_time()
      move_marker.pose.position = navigator_tools.numpy_to_point(self.aligned_position)
      move_marker.pose.orientation = navigator_tools.numpy_to_quaternion(self.aligned_orientation)
      move_marker.type = Marker.ARROW
      move_marker.text = "Align goal"
      move_marker.ns = "detect_deliver"
      move_marker.id = 2
      move_marker.color.r = 1
      move_marker.color.a = 1
      self.markers.markers.append(move_marker)                

    @txros.util.cancellableInlineCallbacks
    def get_normal(self):
        req = CameraToLidarTransformRequest()
        req.header = self.found_shape.header
        req.point = Point()
        req.point.x = self.found_shape.CenterX
        req.point.y = self.found_shape.CenterY
        req.tolerance = self.normal_approx_tolerance_proportion*self.found_shape.img_width
        self.normal_res = yield self.cameraLidarTransformer(req)
        if self.normal_res.success:
            transformObj = yield self.navigator.tf_listener.get_transform('/enu', '/'+req.header.frame_id)
            self.enunormal = transformObj.transform_vector(navigator_tools.rosmsg_to_numpy(self.normal_res.normal))
            self.enupoint = transformObj.transform_point(navigator_tools.rosmsg_to_numpy(self.normal_res.closest))
            target_marker = Marker()
            target_marker.scale.x = 0.1;
            target_marker.scale.y = 0.5;
            target_marker.scale.z = 0.5;
            target_marker.header.frame_id = "enu"
            target_marker.action = Marker.ADD;
            target_marker.header.stamp = self.navigator.nh.get_time()
            target_marker.points.append(navigator_tools.numpy_to_point(self.enupoint))
            target_marker.points.append(navigator_tools.numpy_to_point(self.enupoint+self.enunormal))
            target_marker.type = Marker.ARROW
            #target_marker.text = "Shooter Target Pose"
            target_marker.ns = "detect_deliver"
            target_marker.id = 1
            target_marker.color.r = 1
            target_marker.color.a = 1
            self.markers.markers.append(target_marker)
        defer.returnValue(self.normal_res.success)

    @txros.util.cancellableInlineCallbacks
    def shoot_all_balls(self):
        for i in range(1):
            goal = yield self.shooterLoad.send_goal(ShooterDoAction())
            print "LOAD", i
            res = yield goal.get_result()
            goal = yield self.shooterFire.send_goal(ShooterDoAction())
            print "FIRE", i
            res = yield goal.get_result()

    @txros.util.cancellableInlineCallbacks
    def find_and_shoot(self):
        yield self.navigator.vision_proxies["get_shape"].start()
        yield self.set_shape_and_color()  # Get correct goal shape/color from params
        yield self.get_waypoint()  # Get waypoint of shooter target
        yield self.circle_search()  # Go to waypoint and circle until target found
        yield self.align_to_target()
        yield self.offset_for_target()  # Move a little bit forward to be centered to target
        yield self.shoot_all_balls()
        yield self.navigator.vision_proxies["get_shape"].stop()

@txros.util.cancellableInlineCallbacks
def main(navigator):
    print "Starting Detect Deliver Mission"
    mission = DetectDeliverMission(navigator)
    yield mission.find_and_shoot()
    print "End of Detect Deliver Mission"
