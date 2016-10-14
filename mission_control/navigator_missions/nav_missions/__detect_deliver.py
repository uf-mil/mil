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

        # image_sub = navigator_tools.Image_Subscriber("/right/right/image_raw")
        # camera_info = image_sub.wait_for_camera_info()
        # self.camera = PinholeCameraModel()
        # self.camera.fromCameraInfo(camera_info)

    # def lidar_to_camera(self, lidar_points, t=None): #This may not be needed
    #     #find angle to yaw to
    #     self.navigator.tf_listener.waitForTransform('/right_right_cam', '/velodyne', t, rospy.Duration(2))
    #     trans, rot = self.navigator.tf_listener.lookupTransform('/right_right_cam', '/velodyne', t)

    #     t_mat = np.hstack((tf.transformations.quaternion_matrix(rot)[:3, :3], np.array(trans).reshape(3, 1)))
    #     cam_points = [t_mat.dot(np.append(point, 1)) for point in enu_points]
    #     for point in cam_points:
    #         if point[2] < 0:
    #             # If the point is behind us, ignore
    #             #print "Behind us"
    #             continue

    #         uv_point = np.array(self.camera.project3dToPixel(point), dtype=np.uint32)
    #     return "ha"

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
        pattern = self.navigator.move.circle_point(navigator_tools.rosmsg_to_numpy(self.waypoint_res.object.position), radius=10,theta_offset=1.57)
        searcher = self.navigator.search(vision_proxy='get_shape', search_pattern=pattern, Shape=self.Shape, Color=self.Color)
        yield searcher.start_search(spotings_req=1, speed=1)


    @txros.util.cancellableInlineCallbacks
    def align_perpendicular(self):
        #Allign perpendicular to the face of the target
        print "Aligning perpendicular"


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
           req.tolerance = 10
           #print req
           res = yield self.cameraLidarTransformer(req)
           if res.success:
            transformObj = yield self.navigator.tf_listener.get_transform('/enu', '/right_right_cam', self.found_shape.header.stamp)
            enunormal = transformObj.transform_vector(navigator_tools.rosmsg_to_numpy(res.normal));
            enupoint = transformObj.transform_point(navigator_tools.rosmsg_to_numpy(res.transformed[0]));
            print "POINT = ", enupoint
            print "VECTOR = ", enunormal
            enumove = enupoint+10*enunormal #moves 10 meters away
            print "MOVING TO= ", enumove
            position, rot = yield self.navigator.tx_pose

            print "POSITION= ",position
            print "ROT= ",rot
           

            gotohead = trns.quaternion_from_euler(0,0,np.arccos(enunormal[1])) #Align perpindicular
            print "GOTOHEAD= ", gotohead


            yield self.navigator.move.set_position(enumove).set_orientation(gotohead).go()
            return
            # yield navigator.set_position(n)
            # trans, rot = self.navigator.tf_listener.lookupTransform('/enu', '/right_right_cam', self.resp.symbol.header.stamp)
            # t_mat = np.hstack((tf.transformations.quaternion_matrix(rot)[:3, :3], np.array(trans).reshape(3, 1)))
            # enu_points = [t_mat.dot(res.normal)]
            # print "NORMAL = ", enu_points
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
        for i in range(3):
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
        print "Starting"
        yield self.shootAllBalls()

@txros.util.cancellableInlineCallbacks
def main(navigator):
    mission = DetectDeliverMission(navigator)
    yield mission.findAndShoot()
