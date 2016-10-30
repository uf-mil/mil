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
import navigator_tools


class DetectDeliverMission:
    # Note, this will be changed when the shooter switches to actionlib
    resp = None

    def __init__(self, navigator):
        self.navigator = navigator
        self.cameraLidarTransformer = navigator.nh.get_service_client(
            "/camera_lidar_transformer/transform_camera", CameraToLidarTransform)
        self.shooterLoad = txros.action.ActionClient(
            self.navigator.nh, '/shooter/load', ShooterDoAction)
        self.shooterFire = txros.action.ActionClient(
            self.navigator.nh, '/shooter/fire', ShooterDoAction)


    @txros.util.cancellableInlineCallbacks
    def set_shape_and_color(self):
        # Use params to get shape and color to look for
        self.Shape = yield self.navigator.nh.get_param("/mission/detect_deliver/Shape")
        self.Color = yield self.navigator.nh.get_param("/mission/detect_deliver/Color")
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
        yield self.navigator.move.look_at(navigator_tools.rosmsg_to_numpy(self.waypoint_res.objects[0].position)).go()
        pattern = self.navigator.move.circle_point(navigator_tools.rosmsg_to_numpy(
            self.waypoint_res.objects[0].position), radius=10,  theta_offset=1.57)
        searcher = self.navigator.search(
            vision_proxy='get_shape', search_pattern=pattern, Shape=self.Shape, Color=self.Color)
        yield searcher.start_search(timeout=300, spotings_req=5, move_type="skid")
        print "Ended Circle Search"

    @txros.util.cancellableInlineCallbacks
    def align_to_target(self):
        while True:
            found = yield self.is_found()
            if (found):
                req = CameraToLidarTransformRequest()
                req.header = self.found_shape.header
                req.point = Point()
                req.point.x = self.found_shape.CenterX
                req.point.y = self.found_shape.CenterY
                req.tolerance = 15
                # ~print "Sending transform request: ", req
                res = yield self.cameraLidarTransformer(req)
                # ~print res
                if res.success:
                    # ~print "SUCCESS IN RAY LIDAR"
                    # ~print "TIME: ", self.navigator.nh.get_time()
                    # ~print "STAMP: ", self.found_shape.header.stamp
                    # ~print "FRAME ", req.header.frame_id
                    transformObj = yield self.navigator.tf_listener.get_transform('/enu', '/'+req.header.frame_id)
                    # ~print "Did transform"
                    enunormal = transformObj.transform_vector(
                        navigator_tools.rosmsg_to_numpy(res.normal))
                    enupoint = transformObj.transform_point(
                        navigator_tools.rosmsg_to_numpy(res.closest))
                    #print "DID TRANSFORM ", transformObj
                    #while(enunormal[2] > .75):
                       # print "Loop"
                       # res = yield self.cameraLidarTransformer(req)
                        #transformObj = yield self.navigator.tf_listener.get_transform('/enu',req.header.frame_id, self.found_shape.header.stamp)
                        #enunormal = transformObj.transform_vector(
                         #   navigator_tools.rosmsg_to_numpy(res.normal))
                        #enupoint = transformObj.transform_point(
                         #   navigator_tools.rosmsg_to_numpy(res.closest))
                    
                    print "POINT = ", enupoint
                    print "VECTOR = ", enunormal
                    enumove = enupoint + 5 * enunormal  # moves x meters away
                    print "MOVING TO= ", enumove
                    position, rot = yield self.navigator.tx_pose

                    print "ROT= ", trns.euler_from_quaternion(rot)

                    angle = np.arctan2(enunormal[1], -enunormal[0])

                    enuyaw = trns.quaternion_from_euler(
                        0, 0, angle)  # Align perpindicular
                    print "ROTATING TO= ", trns.euler_from_quaternion(enuyaw)

                    print "MOVING!"
                    yield self.navigator.move.set_position(enumove).set_orientation(enuyaw).go()
                    # return
                    defer.returnValue(None)
                    print "DISTANCE = ", res.distance
                else:
                    print "Error: ", res.error
            else:
                print "Not found"

    @txros.util.cancellableInlineCallbacks
    def offset_for_target(self):
        print "Moving forward to align with large target"
        # Move off of center of shape to be centered with large target
        yield self.navigator.move.forward(0.33)
        # ~self.navigator.move.backward(0.5) #Move off of center of shape to be centered with small target

    @txros.util.cancellableInlineCallbacks
    def is_found(self):
        self.resp = yield self.navigator.vision_proxies["get_shape"].get_response(Shape=self.Shape, Color=self.Color)
        print "Resp ",self.resp
        if self.resp.found:
            self.found_shape = self.resp.shapes.list[0]
        defer.returnValue(self.resp.found)

    @txros.util.cancellableInlineCallbacks
    def shoot_all_balls(self):
        for i in range(1):
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
    def find_and_shoot(self):
        yield self.navigator.vision_proxies["get_shape"].start()
        yield self.set_shape_and_color()  # Get correct goal shape/color from params
        yield self.get_waypoint()  # Get waypoint of shooter target
        yield self.circle_search()  # Go to waypoint and circle until target found
        yield self.align_to_target()
        yield self.offset_for_target()  # Move a little bit forward to be centered to target
        print "Starting to fire"
        yield self.shoot_all_balls()
        yield self.navigator.vision_proxies["get_shape"].stop()

@txros.util.cancellableInlineCallbacks
def main(navigator):
    print "Starting Detect Deliver Mission"
    mission = DetectDeliverMission(navigator)
    yield mission.find_and_shoot()
    print "End of Detect Deliver Mission"
