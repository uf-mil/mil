#!/usr/bin/env python
from __future__ import division
import txros
import std_srvs.srv
import numpy as np
import tf
import tf.transformations as trns
from navigator_msgs.msg import ShooterDoAction, ShooterDoActionGoal
from navigator_msgs.srv import ObjectDBSingleQuery, ObjectDBSingleQueryRequest, CameraToLidarTransform,CameraToLidarTransformRequest
from geometry_msgs.msg import Point
from twisted.internet import defer
from image_geometry import PinholeCameraModel
import navigator_tools

class DetectDeliverMission:
    #Note, this will be changed when the shooter switches to actionlib
    center_error_threshold = 0.1
    width_proportion = 0.15 #Desired proportion of frame width symbol is present in
    width_error_threshold = 0.03
    resp = None


    def __init__(self, navigator):
        # tf_listener = navigator.tf.TransformListener(navigator.nh, history_length=genpy.Duration(1)) # short history length so that we cover history being truncated
        self.center_error = 1
        self.width_error = 1
        self.navigator = navigator
        self.getShooterWaypoint = navigator.nh.get_service_client("/database/single", ObjectDBSingleQuery)
        self.cameraLidarTransformer = navigator.nh.get_service_client("/camera_lidar_transformer/transform_camera",CameraToLidarTransform)
        self.shooterLoad = txros.action.ActionClient(self.navigator.nh, '/shooter/load', ShooterDoAction)
        self.shooterFire = txros.action.ActionClient(self.navigator.nh, '/shooter/fire', ShooterDoAction)

    @txros.util.cancellableInlineCallbacks
    def set_shape_and_color(self):
        #Use params to get shape and color to look for
        self.Shape = yield self.navigator.nh.get_param("/mission/detect_deliver/Shape")
        self.Color = yield self.navigator.nh.get_param("/mission/detect_deliver/Color")
        print "Looking for ",self.Color, " ",self.Shape

    @txros.util.cancellableInlineCallbacks
    def get_waypoint(self):
        res = yield self.getShooterWaypoint(ObjectDBSingleQueryRequest(name="shooter"))
        if not res.found:
            print "Waypoint not found in database, exiting"
            raise Exception('Waypoint not found')
        self.waypoint_res = res


    @txros.util.cancellableInlineCallbacks
    def circle_search(self):
        print "Starting circle search"
        pattern = self.navigator.move.circle_point(navigator_tools.rosmsg_to_numpy(self.waypoint_res.object.position), radius=8,theta_offset=1.57)
        searcher = self.navigator.search(vision_proxy='get_shape', search_pattern=pattern, Shape=self.Shape, Color=self.Color)
        yield searcher.start_search(timeout=300,spotings_req=5, speed=1)
        print "Ended Circle Search"


    @txros.util.cancellableInlineCallbacks
    def align_to_target(self):
       while True:
         found = yield self.is_found()
         if (found):
           req = CameraToLidarTransformRequest()
           req.header.stamp = self.found_shape.header.stamp
           req.point = Point()
           req.point.x = self.found_shape.CenterX
           req.point.y = self.found_shape.CenterY
           req.tolerance = 15
           #print req
           res = yield self.cameraLidarTransformer(req)
           if res.success:
            transformObj = yield self.navigator.tf_listener.get_transform('/enu', '/right_right_cam', self.found_shape.header.stamp)
            enunormal = transformObj.transform_vector(navigator_tools.rosmsg_to_numpy(res.normal));
            enupoint = transformObj.transform_point(navigator_tools.rosmsg_to_numpy(res.closest));
            print "POINT = ", enupoint
            print "VECTOR = ", enunormal
            enumove = enupoint+5*enunormal #moves x meters away
            print "MOVING TO= ", enumove
            position, rot = yield self.navigator.tx_pose

            print "ROT= ",rot

            enuyaw = trns.quaternion_from_euler(0,0,np.arccos(enunormal[0])) #Align perpindicular
            print "ROTATING TO= ", enuyaw

            print "MOVING!"
            yield self.navigator.move.set_position(enumove).set_orientation(enuyaw).yaw_left(3.1415).go()
            #return
            defer.returnValue(None)
            print "DISTANCE = ",res.distance
           else:
            print "Error: ",res.error
         else:
           print "Not found"
         

    @txros.util.cancellableInlineCallbacks
    def center_to_target(self):
        if not (yield self.is_centered()):
            if self.center_error < 0:
                print "Turning Left"
                self.navigator.move.yaw_left(180,"deg").go()
            elif self.center_error > 0:
                print "Turning Right"
                self.navigator.move.yaw_right(180,"deg").go()
            while not (yield self.is_centered()):
                print "...Centering on target"


    @txros.util.cancellableInlineCallbacks
    def offset_for_target(self):
        print "Moving forward to align with large target"
        yield self.navigator.move.forward(0.33) #Move off of center of shape to be centered with large target
        # ~self.navigator.move.backward(0.5) #Move off of center of shape to be centered with small target



    @txros.util.cancellableInlineCallbacks
    def is_found(self):
        self.resp = yield self.navigator.vision_proxies["get_shape"].get_response(Shape=self.Shape, Color=self.Color)
        if self.resp.found:
          self.found_shape = self.resp.shapes.list[0]
        defer.returnValue(self.resp.found)


    @txros.util.cancellableInlineCallbacks
    def is_centered(self):
        if not (yield self.is_found()):
            print "Shape not found"
            # raise Exception('Shape not found')
            defer.returnValue(False)
        center = self.found_shape.CenterX / self.found_shape.img_width
        self.center_error = center - 0.5
        print "Center ", center
        defer.returnValue(abs(self.center_error) < self.center_error_threshold)


    @txros.util.cancellableInlineCallbacks
    def shootAllBalls(self):
        for i in range(4):
            # ~time.sleep(3)
            # ~self.shooterLoad(std_srvs.srv.TriggerRequest())
            # ~time.sleep(5)
            # ~self.shooterFire(std_srvs.srv.TriggerRequest())

            goal = yield self.shooterLoad.send_goal(ShooterDoAction())
            print "LOAD", i
            res = yield goal.get_result()
            goal = yield self.shooterFire.send_goal(ShooterDoAction())
            print "FIRE", i
            res = yield goal.get_result()


    @txros.util.cancellableInlineCallbacks
    def findAndShoot(self):
        yield self.set_shape_and_color() #Get correct goal shape/color from params
        yield self.get_waypoint() #Get waypoint of shooter target
        yield self.circle_search() #Go to waypoint and circle until target found
        yield self.align_to_target()
        yield self.offset_for_target() #Move a little bit forward to be centered to target
        print "Starting to fire"
        yield self.shootAllBalls()

@txros.util.cancellableInlineCallbacks
def main(navigator):
    print "Starting Detect Deliver Mission"
    mission = DetectDeliverMission(navigator)
    yield mission.findAndShoot()
    print "End of Detect Deliver Mission"
