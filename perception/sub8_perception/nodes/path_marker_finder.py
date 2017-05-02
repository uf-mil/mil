#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Header
from std_srvs.srv import SetBool, SetBoolResponse
from sub8_msgs.srv import VisionRequestResponse, VisionRequest, VisionRequest2D, VisionRequest2DResponse, VisionRequest, VisionRequestResponse
from geometry_msgs.msg import PoseStamped
from mil_ros_tools import numpy_quat_pair_to_pose, numpy_to_point, numpy_matrix_to_quaternion, numpy_to_quaternion
from mil_ros_tools import Image_Publisher, Image_Subscriber
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker

__author__ = "Kevin Allen"

class PathMarkerFinder():
    """
    Node which finds orange path markers in image frame, 
    estimates the 2d and 3d position/orientation of marker
    and returns this estimate when service is called.
    
    Unit tests for this node is in test_path_marker.py
    
    Finding the marker works as follows:
    * blur image
    * threshold image mostly for highly saturated, orange/yellow/red objects
    * run canny edge detection on thresholded image
    * find contours on edge frame
    * filter contours to find those that may be contours by:
      * checking # of sides in appox polygon
      * checking ratio of length/width close to known model
    * estimates 3D pose using cv2.solvePnP with known object demensions and camera model
    
    TODO: Implement Kalman filter to smooth pose estimate / eliminate outliers
    """
    # Model of four corners of path marker, centered around 0 in meters
    LENGTH = 0.6096 # Longer side of path marker in meters
    WIDTH = 0.0762 # Shorter Side
    PATH_MARKER = np.array([[LENGTH,  -WIDTH, 0],
                            [LENGTH,  WIDTH,  0],
                            [-LENGTH,  WIDTH, 0],
                            [-LENGTH, -WIDTH, 0]], dtype=np.float)

    # Scale model of marker as a cv2 contour, for use in cv2.matchShape
    PATH_MARKER_2D = np.array([[[0, 0]],
                               [[WIDTH*10000, 0]],
                               [[WIDTH*10000, LENGTH*10000]],
                               [[0, LENGTH*10000]]], dtype=np.int)

    # Coordinate axes for debugging image
    REFERENCE_POINTS = np.array([[0, 0, 0],
                                 [0.3, 0, 0],
                                 [0, 0.3, 0],
                                 [0, 0, 0.3]], dtype=np.float)
    def __init__(self):
        self.debug_gui = False
        self.last2d = None
        self.last3d = None
        self.enabled = False
        self.cam = None
        self.last_image = None
        self.last_found_time = None

        # Constants from launch config file
        self.debug_ros = rospy.get_param("~debug_ros", True)
        self.canny_low = rospy.get_param("~canny_low", 100)
        self.canny_ratio = rospy.get_param("~canny_ratio", 3.0)
        self.thresh_hue_high = rospy.get_param("~thresh_hue_high", 60)
        self.thresh_saturation_low = rospy.get_param("~thresh_satuation_low", 100)
        self.min_contour_area = rospy.get_param("~min_contour_area", 100)
        self.length_width_ratio_err = rospy.get_param("~length_width_ratio_err", 0.2)
        self.approx_polygon_thresh = rospy.get_param("~approx_polygon_thresh", 10)
        self.shape_match_thresh = rospy.get_param("~shape_match_thresh", 0.4)
        self.min_found_count = rospy.get_param("~min_found_count", 10)
        self.timeout_seconds = rospy.get_param("~timeout_seconds", 2.0)
        camera = rospy.get_param("~marker_camera", "/camera/down/left/image_rect_color")

        self.state_size = 8 # X, Y, Z, THETA, vx, vy, vz, vtheta
        self.measurement_size = 4

        self.filter = cv2.KalmanFilter(self.state_size, self.measurement_size)
        self.filter.transitionMatrix = 1.* np.eye(self.state_size, self.state_size, dtype=np.float32)
        self.filter.measurementMatrix = 1. * np.array([[1, 0, 0, 0, 0, 0, 0, 0],
                                                       [0, 1, 0, 0, 0, 0, 0, 0],
                                                       [0, 0, 1, 0, 0, 0, 0, 0],
                                                       [0, 0, 0, 1, 0, 0, 0, 0]], dtype=np.float32)
        self.filter.processNoiseCov = 1e-5 * np.eye(self.state_size, dtype=np.float32)
        self.filter.measurementNoiseCov = 1e-4 * np.eye(self.measurement_size, dtype=np.float32)
        self.filter.errorCovPost = 1.* np.eye(self.state_size, dtype=np.float32)
        self._clear_filter(None)

        if self.debug_ros:
            self.debug_pub = Image_Publisher("~debug_image")
            self.markerPub = rospy.Publisher('~path_marker_visualization', Marker, queue_size=10)
        self.service2D = rospy.Service('/vision/path_marker/2D', VisionRequest2D, self.cb_2d)
        self.service3D = rospy.Service('/vision/path_marker/pose', VisionRequest, self.cb_3d)
        self.toggle = rospy.Service('/vision/path_marker/enable', SetBool, self.enable_cb)
        self.image_sub = Image_Subscriber(camera, self.img_cb)
        camera_info = self.image_sub.wait_for_camera_info()
        self.cam = PinholeCameraModel()
        self.cam.fromCameraInfo(camera_info)

    def sendDebugMarker(self):
        '''
        Sends a rviz marker in the camera frame with the estimated pose of the path marker.
        This marker is a scaled cube with the demensions and color of the actual marker.
        Only called if debug_ros param == True
        '''
        m = Marker()
        m.header.frame_id = self.cam.tfFrame()
        m.header.stamp = self.last_found_time
        m.ns = "path_markers"
        m.id = 0
        m.type = 1
        m.action = 0
        m.pose.position = numpy_to_point(self.last3d[0])
        m.pose.orientation = numpy_to_quaternion(self.last3d[1])
        # Real demensions of path marker
        m.scale.x = 1.2192
        m.scale.y = 0.1524
        m.scale.z = 0.05
        m.color.r = 0.0
        m.color.g = 0.5
        m.color.b = 0.0
        m.color.r = 1.0
        m.color.a = 1.0
        m.lifetime = rospy.Duration(0)
        self.markerPub.publish(m)

    def enable_cb(self, x):
        if x.data != self.enabled:
            self._clear_filter(None)
        self.enabled = x.data
        return SetBoolResponse(success=True)

    def sortRect(self, rect):
        '''
        Given a contour of 4 points, returns the same 4 points sorted in a known way.
        Used so that indicies of contour line up to that in model for cv2.solvePnp
        p[0] = Top left corner of marker
        p[1] = Top right corner of marker
        p[2] = Bottom right corner of marker
        p[3] = Bottom left cornet of marker
        '''
        def sort_contour_y(c):
            return c[0][1]
        def sort_contour_x(c):
            return c[0][0]
        sorted_y = np.array(sorted(rect, key=sort_contour_y))
        sorted_x = np.array(sorted(rect, key=sort_contour_x))

        sorted_rect = None
        if (np.linalg.norm(sorted_y[0] - sorted_y[1]) > np.linalg.norm(sorted_y[0] - sorted_y[2]) or
                np.linalg.norm(sorted_y[0] - sorted_y[1]) > np.linalg.norm(sorted_y[0] - sorted_y[3])):
            if sorted_x[1][0][1] > sorted_x[0][0][1]:
                sorted_x[0], sorted_x[1] = sorted_x[1].copy(), sorted_x[0].copy()
            if sorted_x[2][0][1] > sorted_x[3][0][1]:
                sorted_x[2], sorted_x[3] = sorted_x[3].copy(), sorted_x[2].copy()
            sorted_rect =  sorted_x
        else:
            if sorted_y[0][0][0] > sorted_y[1][0][0]:
                sorted_y[0], sorted_y[1] = sorted_y[1].copy(), sorted_y[0].copy()
            if sorted_y[3][0][0] > sorted_y[2][0][0]:
                sorted_y[3], sorted_y[2] = sorted_y[2].copy(), sorted_y[3].copy()
            sorted_rect = sorted_y
        for i, pixel in enumerate(sorted_rect):
            center = (int(pixel[0][0]), int(pixel[0][1]))
            cv2.circle(self.last_image, center, 5, (255, 0, 0), -1)
            cv2.putText(self.last_image, str(i), center, cv2.FONT_HERSHEY_SCRIPT_COMPLEX,1, (0,0,255))
        return sorted_rect

    def cb_3d(self, req):
        res = VisionRequestResponse()
        dt = (self.image_sub.last_image_time - self.last_found_time).to_sec()
        if dt <= 0 or dt > self.timeout_seconds:
            res.found = False
        elif (self.last3d == None or not self.enabled):
            res.found = False
        else:
            res.pose.header.frame_id = self.cam.tfFrame()
            res.pose.header.stamp = self.last_found_time
            res.pose.pose.position = numpy_to_point(self.last3d[0])
            res.pose.pose.orientation = numpy_to_quaternion(self.last3d[1])
            res.found = True
        return res
    
    def cb_2d(self, req):
        res = VisionRequest2DResponse()
        if (self.last2d == None or not self.enabled):
            res.found = False
        else:
            res.header.frame_id = self.cam.tfFrame()
            res.header.stamp = self.last_found_time
            res.pose.x = self.last2d[0][0]
            res.pose.y = self.last2d[0][1]
            res.pose.theta = self.last2d[1]
            res.found = True
        return res

    def _update_transition_matrix(self, dt):
        self.filter.transitionMatrix[0][4] = dt
        self.filter.transitionMatrix[1][5] = dt
        self.filter.transitionMatrix[2][6] = dt
        self.filter.transitionMatrix[3][7] = dt

    def _clear_filter(self, state):
        self.found_count = 0
        self.found = False
        self.last3d = None
        self.filter.errorCovPre = 1. * np.eye(self.state_size, dtype=np.float32)
        if state is not None:
            (x, y, z, theta) = state
            state = np.array([[x], [y], [z], [theta], [0], [0], [0], [0]], dtype=np.float32)
            self.filter.statePost = state

    def _update_kf(self, (x, y, z, theta)):
        if self.last_found_time is None:
            self._clear_filter((x, y, z, theta))
            return
        dt = (self.image_sub.last_image_time - self.last_found_time).to_sec()
        if dt <= 0 or dt > self.timeout_seconds:
            rospy.logwarn("Timed out since last saw marker, resetting")
            self._clear_filter((x, y, z, theta))
            return
        if self.last3d is not None:
            if np.linalg.norm(np.array([x, y, z], dtype=np.float32) - self.last3d[0]) > 15:
                self._clear_filter((x, y, z, theta))
                rospy.logwarn("Too far apart, resetting")
                return

        self.found_count += 1
        self._update_transition_matrix(dt)
        measurement = 1.* np.array([x, y, z, theta], dtype=np.float32)
        predict = self.filter.predict()
        estimated = self.filter.correct(measurement)
        if self.found_count > self.min_found_count:
              self.last3d = ((estimated[0][0], estimated[1][0], estimated[2][0]),
                              quaternion_from_euler(0.0, 0.0, estimated[3][0]))
              if not self.found:
                  rospy.loginfo("Marker Found")
              self.found = True
              if self.debug_ros:
                  refs, _ = cv2.projectPoints(np.array([[ estimated[0][0], estimated[1][0], estimated[2][0] ]] ), (0, 0, 0), (0,0,0), self.cam.intrinsicMatrix(), np.zeros((5,1)))
                  center = (int(refs[0][0][0]), int(refs[0][0][1]))
                  cv2.circle(self.last_image, center, 8, (0, 0, 255), -1)
                  text = str(np.degrees(estimated[3][0]))+"deg"
                  cv2.putText(self.last_image, text, center, cv2.FONT_HERSHEY_SCRIPT_COMPLEX,1, (0,0,255))
                  self.sendDebugMarker()

    def get_3d_pose(self, p):
        i_points = np.array((p[0][0], p[1][0], p[2][0], p[3][0]),dtype=np.float)
        retval, rvec, tvec =  cv2.solvePnP(PathMarkerFinder.PATH_MARKER, i_points, self.cam.intrinsicMatrix(), np.zeros((5,1)))
        if tvec[2] < 0.3 :
            return False
        self._update_kf((tvec[0], tvec[1], tvec[2], self.last2d[1]))
        if self.debug_ros:
            refs, _ = cv2.projectPoints(PathMarkerFinder.REFERENCE_POINTS, rvec, tvec, self.cam.intrinsicMatrix(), np.zeros((5,1)))
            refs = np.array(refs, dtype=np.int)
            cv2.line(self.last_image, (refs[0][0][0], refs[0][0][1]), (refs[1][0][0], refs[1][0][1]), (0, 0, 255)) # X axis refs
            cv2.line(self.last_image, (refs[0][0][0], refs[0][0][1]), (refs[2][0][0], refs[2][0][1]), (0, 255, 0)) # Y axis ref
            cv2.line(self.last_image, (refs[0][0][0], refs[0][0][1]), (refs[3][0][0], refs[3][0][1]), (255, 0, 0)) # Z axis ref
        return True
    
    def get_2d_pose(self, r):
        '''
        Given a sorted 4 sided polygon, stores the centriod and angle
        for the next service call to get 2dpose.
        returns False if dx of polygon is 0, otherwise True
        '''
        top_center = (r[1][0]+r[0][0])/2.0
        bot_center = (r[2][0]+r[3][0])/2.0
        center = bot_center + (top_center - bot_center)/2.0
        dy = top_center[1] - bot_center[1]
        dx = top_center[0] - bot_center[0]
        if dx == 0:
            rospy.logerr("Contour dx is 0, strange...")
            return False

        angle = np.arctan(dy / dx)
        self.last2d = (center, angle)
        return True

    def valid_contour(self, contour):
        '''
        Does various tests to filter out contours that are clearly not
        a valid path marker.
        * run approx polygon, check that sides == 4
        * find ratio of length to width, check close to known ratio IRL
        '''
        if cv2.contourArea(contour) < self.min_contour_area:
            return False
        match_shapes = cv2.matchShapes(contour, PathMarkerFinder.PATH_MARKER_2D, 3, 0.0)
        if match_shapes > self.shape_match_thresh :
            return False
        # Checks that contour is 4 sided
        polygon = cv2.approxPolyDP(contour, self.approx_polygon_thresh, True)
        if len(polygon) != 4:
            #rospy.loginfo("Polygon not 4 sided ({}), throwing out".format(len(polygon)))
            return False
        rect = self.sortRect(polygon)
        if not self.get_2d_pose(rect):
            return False
        if not self.get_3d_pose(rect):
            return False
        return True

    def get_edges(self):
        '''
        Proccesses latest image to find edges by:
        blurring and thresholding for highly saturated orangish objects
        then runs canny on threshold images and returns canny's edges
        '''
        blur = cv2.blur(self.last_image, (5,5))
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv, (0, self.thresh_saturation_low, 0), (self.thresh_hue_high, 255, 255))
        return cv2.Canny(thresh, self.canny_low, self.canny_low*self.canny_ratio)

    def img_cb(self, img):
        if not self.enabled or self.cam == None:
            return
        self.last_image = img
        edges = self.get_edges()
        _, contours, _ = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        # Check if each contour is valid
        for idx, c in enumerate(contours):
            if self.valid_contour(c):
                self.last_found_time = self.image_sub.last_image_time
                if self.debug_ros:
                    cv2.drawContours(self.last_image, contours, idx, (0,255,0), 3)
                break
            else:
                if self.debug_ros:
                    cv2.drawContours(self.last_image, contours, idx, (255,0,0), 3)
        if self.debug_ros:
            self.debug_pub.publish(self.last_image)
        if self.debug_gui:
            cv2.imshow("debug", self.last_image)
            cv2.waitKey(5)

if __name__ == '__main__':
    rospy.init_node('path_marker_finder')
    find = PathMarkerFinder()
    rospy.spin()
