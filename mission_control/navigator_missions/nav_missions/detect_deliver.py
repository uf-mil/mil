#!/usr/bin/env python
from __future__ import division
import txros
import std_srvs.srv
import numpy as np
import tf
from navigator_msgs.msg import ShooterDoAction, ShooterDoActionGoal
from navigator_msgs.srv import PerceptionObjectService
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
        self.center_error = 1
        self.width_error = 1
        self.navigator = navigator
        # ~self.shooterLoad = navigator.nh.get_service_client("/shooter/load", std_srvs.srv.Trigger)
        # ~self.shooterFire = navigator.nh.get_service_client("/shooter/fire", std_srvs.srv.Trigger)
        self.getShooterWaypoint = navigator.nh.get_service_cleint("/database/single", navigator_msgs.srv.PerceptionObjectService)
        self.Shape = "CIRCLE"
        self.Color = "RED"
        self.shooterLoad = txros.action.ActionClient(self.navigator.nh, '/shooter/load', ShooterDoAction)
        self.shooterFire = txros.action.ActionClient(self.navigator.nh, '/shooter/fire', ShooterDoAction)
        image_sub = navigator_tools.Image_Subscriber("/right/ight/image_raw")
        camera_info = image_sub.wait_for_camera_info()
        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(camera_info)

    def lidar_to_camera(self, enu_points, t=None): #This may not be needed
        #find angle to yaw to
        self.navigator.tf_listener.waitForTransform('/right/right', '/enu', t, rospy.Duration(2))
        trans, rot = self.navigator.tf_listener.lookupTransform('/right/right', '/enu', t)

        t_mat = np.hstack((tf.transformations.quaternion_matrix(rot)[:3, :3], np.array(trans).reshape(3, 1)))
        cam_points = [t_mat.dot(np.append(point, 1)) for point in enu_points]
        for point in cam_points:
            if point[2] < 0:
                # If the point is behind us, ignore
                #print "Behind us"
                continue

            uv_point = np.array(self.camera.project3dToPixel(point), dtype=np.uint32)
        return "ha"


    def set_shape_and_color(self):
        #Use params to get shape and color to look for
        self.Shape = self.navigator.nh.get_param("/mission/detect_deliver/Shape")
        self.Color = self.navigator.nh.get_param("/mission/detect_deliver/Color")


    @txros.util.cancellableInlineCallbacks
    def get_waypoint(self):
        res = yield self.getShooterWaypoint(PerceptionObjectServiceRequest(name="detect_deliver_target"))
        if not res.found:
            print "Waypoint not found in database, exiting"
            raise Exception('Waypoint not found')
        self.waypoint_res = res


    @txros.util.cancellableInlineCallbacks
    def circle_search(self):
        shape = self.nh.get_param()
        pattern = navigator.move.circle_point(self.waypoint_res.prev_pose.point, radius=10)
        searcher = navigator.search(vision_proxy='get_shape', search_pattern=pattern, Shape=self.Shape, Color=self.Color)
        yield searcher.start_search(spotings_req=1, speed=1)


    @txros.util.cancellableInlineCallbacks
    def align_perpendicular(self):
        #Allign perpendicular to the face of the target
        print "Aligning perpendicular"


    @txros.util.cancellableInlineCallbacks
    def align_distance(self):
        #Allign the correct distance from the target plane
        print "Aligning distance"


    @txros.util.cancellableInlineCallbacks
    def center_to_target(self):
        if not (yield self.is_centered()):
            if self.center_error < 0:
                print "Turning Left"
                self.navigator.move.forward(10).go()
            elif self.center_error > 0:
                print "Turning Right"
                self.navigator.move.backward(10).go()
            while not (yield self.is_centered()):
                print "...Centering on target"


    @txros.util.cancellableInlineCallbacks
    def offset_for_target(self):
        yield self.navigator.move.forward(0.33) #Move off of center of shape to be centered with large target
        # ~self.navigator.move.backward(0.5) #Move off of center of shape to be centered with small target


    def bounding_rect(self, points):
        maxX = 0
        minX = 2000
        maxY = 0
        minY = 2000
        for i in range(len(points)):
            if points[i].x > maxX:
                maxX = points[i].x
            if points[i].y > maxY:
                maxY = points[i].y
            if points[i].x < minX:
                minX = points[i].x
            if points[i].y < minY:
                minY = points[i].y
        return np.array([maxX, maxY, minX, minY])


    @txros.util.cancellableInlineCallbacks
    def is_found(self):
        self.resp = yield self.navigator.vision_request("get_shape", Shape=self.Shape, Color=self.Color)
        yield self.resp.found and self.resp.symbol.img_width != 0


    @txros.util.cancellableInlineCallbacks
    def is_centered(self):
        if not (yield self.is_found()):
            print "Shape not found"
            raise Exception('Shape not found')
            return False
        center = self.resp.symbol.CenterX / self.resp.symbol.img_width
        self.center_error = center - 0.5
        print "Center ", center
        yield abs(self.center_error) < self.center_error_threshold


    @txros.util.cancellableInlineCallbacks
    def shootAllBalls(self):
        for i in range(3):
            # ~time.sleep(3)
            # ~self.shooterLoad(std_srvs.srv.TriggerRequest())
            # ~time.sleep(5)
            # ~self.shooterFire(std_srvs.srv.TriggerRequest())
            self.shooterLoad.send_goal(ShooterDoAction())
            res = yield goal.get_result()
            self.shooterFire.send_goal(ShooterDoAction())
            res = yield goal.get_result()


    @txros.util.cancellableInlineCallbacks
    def findAndShoot(self):
        self.set_shape_and_color() #Get correct goal shape/color from params
        yield self.get_waypoint() #Get waypoint of shooter target
        yield self.circle_search() #Go to waypoint and circle until arget found
        yield self.align_perpendicular()
        yield self.align_distance()
        yield self.center_to_target()
        yield self.offset_for_target() #Move a little bit forward to be centered to target
        print "Shooting"
        self.shootAllBalls()

@txros.util.cancellableInlineCallbacks
def main(navigator):
    mission = DetectDeliverMission(navigator)
    yield mission.findAndShoot()
