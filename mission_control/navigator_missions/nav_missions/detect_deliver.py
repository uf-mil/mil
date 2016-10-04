#!/usr/bin/env python
import txros
import std_srvs.srv
import numpy as np
from navigator_msgs.msg import ShooterDoAction, ShooterDoActionGoal
from navigator_msgs.srv import PerceptionObjectService
from __future__ import division
import time
from twisted.internet import defer

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
        self.shooterLoad = navigator.nh.get_service_client("/shooter/load", std_srvs.srv.Trigger)
        self.shooterFire = navigator.nh.get_service_client("/shooter/fire", std_srvs.srv.Trigger)
        self.getShooterWaypoint = navigator.nh.get_service_cleint("/database/single",navigator_msgs.srv.PerceptionObjectService)
        self.Shape = "CIRCLE"
        self.Color = "RED"
        # ~self.shooterLoad = txros.action.ActionClient(self.navigator.nh, '/shooter/load', ShooterDoAction)
        # ~self.shooterFire = txros.action.ActionClient(self.navigator.nh, '/shooter/fire', ShooterDoAction)


    def set_shape_and_color(self):
        #Use params to get shape and color to look for
        self.Shape = navigator.nh.get_param("/mission/detect_deliver/Shape")
        self.Color = navigator.nh.get_param("/mission/detect_deliver/Color")


    @txros.util.cancellableInlineCallbacks
    def getWaypoint(self):
        res = yield getShooterWaypoint(PerceptionObjectServiceRequest(name="detect_deliver_target"))
        if (!res.found):
            print "Waypoint not found in database, exiting"
            raise Exception('Waypoint not found') 
        self.waypoint_res = res

    @txros.util.cancellableInlineCallbacks
    def circleSearch(self):
        shape = self.nh.get_param()
        pattern = navigator.move.circle_point(self.waypoint_res.prev_pose.point, radius=10)
        searcher = navigator.search(vision_proxy='get_shape', search_pattern=pattern,Shape=self.Shape,Color=self.Color)
        yield searcher.start_search(spotings_req=1, speed=1)

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
        self.set_shape_and_color() #Get correct goal shape/color from params
        yield self.getWaypoint() #Get waypoint of shooter target
        yield self.circleSearch() #Go to waypoint and circle until arget found
        
        #Remove, should have already found when mission run
        #if not (yield self.isFound()):
          #self.navigator.move.yaw_left(360, "deg").go()
          #while not (yield self.isFound()):
            #print "...Searching for target"
        #print "Found"
        # ~yield self.navigator.move.go() #Stop moving once shape seen
        # ~if not (yield self.isCentered()):
            # ~if self.center_error < 0:
                # ~print "Turning Left"
                # ~self.navigator.move.yaw_left(180, "deg").go()
            # ~elif self.center_error > 0:
                # ~print "Turning Right"
                # ~self.navigator.move.yaw_right(180, "deg").go()
            # ~while not (yield self.isCentered()):
                # ~print "...Centering on target"
        # ~print "Centered"
        # ~yield self.navigator.move.go() #Stop moving once shape seen
        # ~if not (yield self.isCorrectDistance()):
            # ~if self.width_error < 0:
                # ~print "Moving Towards"
                # ~self.navigator.move.right(50).go()
            # ~elif self.width_error > 0:
                # ~print "Moving Away"
                # ~self.navigator.move.left(50).go()
            # ~while not (yield self.isCorrectDistance()):
                # ~print "...Moving to correct distance"
        # ~yield self.navigator.move.go() #Stop moving once centered
        # ~print "Correct Distance"

        print "Shooting"
        self.shootAllBalls()

@txros.util.cancellableInlineCallbacks
def main(navigator):
    mission = DetectDeliverMission(navigator)
    yield mission.findAndShoot()
