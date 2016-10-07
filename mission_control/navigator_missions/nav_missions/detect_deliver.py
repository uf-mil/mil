#!/usr/bin/env python
import txros
import std_srvs.srv
import tf
import numpy as np
from navigator_msgs.msg import ShooterDoAction, ShooterDoActionGoal
from __future__ import division
import time
from twisted.internet import defer
from image_geometry import PinholeCameraModel
import navigator_tools


class DetectDeliverMission:
    #Note, this will be changed when the shooter switches to actionlib
    shooterLoad = nh.get_service_client("/shooter/load", std_srvs.srv.Trigger)
    shooterFire = nh.get_service_client("/shooter/fire", std_srvs.srv.Trigger)
    center_error_threshold = 0.1
    width_proportion = 0.15 #Desired proportion of frame width symbol is present in
    width_error_threshold = 0.03
    resp = None

    def __init__(self, navigator):
        self.center_error = 1
        self.width_error = 1
        self.navigator = navigator
        image_sub = navigator_tools.Image_Subscriber("/right/ight/image_raw")
        camera_info = image_sub.wait_for_camera_info()
        self.camera = PinholeCameraModel()
        self.camera.fromCameraInfo(camera_info)

        # ~self.shooterLoad = txros.action.ActionClient(self.navigator.nh, '/shooter/load', ShooterDoAction)
        # ~self.shooterFire = txros.action.ActionClient(self.navigator.nh, '/shooter/fire', ShooterDoAction)

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

    def move_infront(self):
        #Get normal transform relative to Camera
        #Multiply normal by a constant of like 10 meters and add it to the (center/plane) point


    def bounding_rect(self,points):
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
    def isFound(self):
        self.resp = yield self.navigator.vision_request("get_shape")
        yield self.resp.found and self.resp.symbol.img_width != 0


    @txros.util.cancellableInlineCallbacks
    def isCentered(self):
        if not (yield self.isFound()):
            yield False
        center = self.resp.symbol.CenterX / self.resp.symbol.img_width
        self.center_error = center - 0.5
        print "Center ", center
        yield abs(self.center_error) < self.center_error_threshold


    @txros.util.cancellableInlineCallbacks
    def isCorrectDistance(self):
        if not (yield self.isFound()):
            yield False
        rect = self.bounding_rect(self.resp.symbol.points)
        width = (rect[0] - rect[2]) / self.resp.symbol.img_width
        self.width_error = width - self.width_proportion
        print "Width ", width
        yield abs(self.width_error) < self.width_error_threshold


    @txros.util.cancellableInlineCallbacks
    def shootAllBalls(self):
        for i in range(3):
            time.sleep(3)
            self.shooterLoad(std_srvs.srv.TriggerRequest())
            time.sleep(5)
            self.shooterFire(std_srvs.srv.TriggerRequest())
            # ~self.shooterLoad.send_goal(ShooterDoAction())
            # ~res = yield goal.get_result()
            # ~self.shooterFire.send_goal(ShooterDoAction())
            # ~res = yield goal.get_result()

    @txros.util.cancellableInlineCallbacks
    def findAndShoot(self):
        #Remove, should have already found when mission run
        #if not (yield self.isFound()):
          #self.navigator.move.yaw_left(360, "deg").go()
          #while not (yield self.isFound()):
            #print "...Searching for target"
        #print "Found"
        yield self.navigator.move.go() #Stop moving once shape seen
        if not (yield self.isCentered()):
            if self.center_error < 0:
                print "Turning Left"
                self.navigator.move.yaw_left(180, "deg").go()
            elif self.center_error > 0:
                print "Turning Right"
                self.navigator.move.yaw_right(180, "deg").go()
            while not (yield self.isCentered()):
                print "...Centering on target"
        print "Centered"
        yield self.navigator.move.go() #Stop moving once shape seen
        if not (yield self.isCorrectDistance()):
            if self.width_error < 0:
                print "Moving Towards"
                self.navigator.move.right(50).go()
            elif self.width_error > 0:
                print "Moving Away"
                self.navigator.move.left(50).go()
            while not (yield self.isCorrectDistance()):
                print "...Moving to correct distance"
        yield self.navigator.move.go() #Stop moving once centered
        print "Correct Distance"

        print "Shooting"
        self.shootAllBalls()

@txros.util.cancellableInlineCallbacks
def main(navigator):
    mission = DetectDeliverMission(navigator)
    yield mission.findAndShoot()
