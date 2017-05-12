#!/usr/bin/env python
import cv2
import numpy as np
import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix
from std_msgs.msg import Header
from std_srvs.srv import SetBool, SetBoolResponse
from sub8_msgs.srv import VisionRequestResponse, VisionRequest, VisionRequest2D, VisionRequest2DResponse, VisionRequest, VisionRequestResponse
from geometry_msgs.msg import PointStamped, Point, Quaternion, Vector3Stamped
from mil_ros_tools import numpy_quat_pair_to_pose, numpy_to_point, numpy_matrix_to_quaternion, numpy_to_quaternion, rosmsg_to_numpy
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

    # Demensions of the prop we made for testing
    # LENGTH = 0.4572
    # WIDTH = 0.05715

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
        self.last_found_time_2D = None
        self.last_found_time_3D = None

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
        self.do_3D = rospy.get_param("~do_3D", True)
        camera = rospy.get_param("~marker_camera", "/camera/down/left/image_rect_color")

        self.tf_listener = tf.TransformListener()

        # Create kalman filter to track 3d position and direction vector for marker in /map frame
        self.state_size = 5 # X, Y, Z, DY, DX
        self.filter = cv2.KalmanFilter(self.state_size, self.state_size)
        self.filter.transitionMatrix = 1.* np.eye(self.state_size, dtype=np.float32)
        self.filter.measurementMatrix = 1. * np.eye(self.state_size, dtype=np.float32)
        self.filter.processNoiseCov = 1e-5 * np.eye(self.state_size, dtype=np.float32)
        self.filter.measurementNoiseCov = 1e-4 * np.eye(self.state_size, dtype=np.float32)
        self.filter.errorCovPost = 1.* np.eye(self.state_size, dtype=np.float32)
        self._clear_filter(None)

        if self.debug_ros:
            self.debug_pub = Image_Publisher("~debug_image")
            self.markerPub = rospy.Publisher('~path_marker_visualization', Marker, queue_size=10)
        self.service2D = rospy.Service('/vision/path_marker/2D', VisionRequest2D, self._vision_cb_2D)
        if self.do_3D:
            self.service3D = rospy.Service('/vision/path_marker/pose', VisionRequest, self._vision_cb_3D)
        self.toggle = rospy.Service('/vision/path_marker/enable', SetBool, self._enable_cb)
        self.image_sub = Image_Subscriber(camera, self._img_cb)
        camera_info = self.image_sub.wait_for_camera_info()
        self.cam = PinholeCameraModel()
        self.cam.fromCameraInfo(camera_info)

    def _send_debug_marker(self):
        '''
        Sends a rviz marker in the camera frame with the estimated pose of the path marker.
        This marker is a scaled cube with the demensions and color of the actual marker.
        Only called if debug_ros param == True
        '''
        if self.last3d is None or not self.found:
            return
        m = Marker()
        m.header.frame_id = '/map'
        m.header.stamp = self.last_found_time_3D
        m.ns = "path_markers"
        m.id = 0
        m.type = 1
        m.action = 0
        # Real demensions of path marker
        m.scale.x = 1.2192
        m.scale.y = 0.1524
        m.scale.z = 0.05
        m.pose.position = numpy_to_point(self.last3d[0])
        m.pose.orientation = numpy_to_quaternion(self.last3d[1])
        m.color.r = 0.0
        m.color.g = 0.5
        m.color.b = 0.0
        m.color.r = 1.0
        m.color.a = 1.0
        m.lifetime = rospy.Duration(self.timeout_seconds)
        self.markerPub.publish(m)

    def _enable_cb(self, x):
        if x.data != self.enabled:
            self._clear_filter(None)
        self.enabled = x.data
        return SetBoolResponse(success=True)

    def _sort_rect(self, rect):
        '''
        Given a contour of 4 points, returns the same 4 points sorted in a known way.
        Used so that indicies of contour line up to that in model for cv2.solvePnp
        p[0] = Top left corner of marker         0--1
        p[1] = Top right corner of marker        |  |
        p[2] = Bottom right corner of marker     |  |
        p[3] = Bottom left cornet of marker      3--2

        The implementation of this is a little bit of magic, and there may be an easier way
        to do this. For now you can just see it as a black box, which takes four corners
        of the path marker and returns the same four corners in a known order
        '''
        rect = np.reshape(rect, (rect.shape[0], 2))
        sorted_x = np.argsort(rect[:,0])
        sorted_y = np.argsort(rect[:,1])
        # Wrap index around from 3 to 0
        def ind_next(i):
            if i >= 3:
              return 0
            return i+1
        # Given two correct corner indexes, get other 2
        def get_two(a, b):
            if a - b == 1: # Ex: (1,0) -> (3,2)
                return (ind_next(ind_next(a)), ind_next(a))
            elif a - b == -1: # Ex: (0, 1) -> (2,3)
                return (ind_next(b), ind_next(ind_next(b)))
            elif a - b == 3:
                return (1,2)
            elif a - b == -3:
                return (2,1)

        # Horizontal orientation
        if np.linalg.norm(rect[sorted_y[0]] - rect[sorted_y[1]]) >  np.linalg.norm(rect[sorted_y[0]] - rect[sorted_y[2]]):
            # If 1 is lower than 0, reverse
            if rect[sorted_x[1]][1] > rect[sorted_x[0]][1]:
                sorted_x[0], sorted_x[1] = sorted_x[1].copy(), sorted_x[0].copy()
            next_two = get_two(sorted_x[0], sorted_x[1])
            sorted_x[2], sorted_x[3] = next_two[0], next_two[1]
            rect = np.array([rect[sorted_x[0]], rect[sorted_x[1]], rect[sorted_x[2]], rect[sorted_x[3]]])
        else:
            # If 1 is to the left of zero, reverse
            if rect[sorted_y[0]][0] > rect[sorted_y[1]][0]:
                sorted_y[0], sorted_y[1] = sorted_y[1].copy(), sorted_y[0].copy()
            next_two = get_two(sorted_y[0], sorted_y[1])
            sorted_y[2], sorted_y[3] = next_two[0], next_two[1]
            rect = np.array([rect[sorted_y[0]], rect[sorted_y[1]], rect[sorted_y[2]], rect[sorted_y[3]]])
        if self.debug_ros:
            for i, pixel in enumerate(rect):
                center = (int(pixel[0]), int(pixel[1]))
                cv2.circle(self.last_image, center, 5, (0, 255, 0), -1)
                cv2.putText(self.last_image, str(i), center, cv2.FONT_HERSHEY_SCRIPT_COMPLEX,1, (0,0,255))
        return rect

    def _vision_cb_3D(self, req):
        res = VisionRequestResponse()
        if self.last_found_time_3D is None or self.image_sub.last_image_time is None:
            res.found = False
            return res
        dt = (self.image_sub.last_image_time - self.last_found_time_3D).to_sec()
        if dt < 0 or dt > self.timeout_seconds:
            res.found = False
        elif (self.last3d == None or not self.enabled):
            res.found = False
        else:
            res.pose.header.frame_id = "/map"
            res.pose.header.stamp = self.last_found_time_3D
            res.pose.pose.position = numpy_to_point(self.last3d[0])
            res.pose.pose.orientation = numpy_to_quaternion(self.last3d[1])
            res.found = True
        return res
    
    def _vision_cb_2D(self, req):
        res = VisionRequest2DResponse()
        if (self.last2d == None or not self.enabled):
            res.found = False
        else:
            res.header.frame_id = self.cam.tfFrame()
            res.header.stamp = self.last_found_time_2D
            res.pose.x = self.last2d[0][0]
            res.pose.y = self.last2d[0][1]
            if self.last2d[1][0] < 0:
                self.last2d[1][0] = -self.last2d[1][0]
                self.last2d[1][1] = -self.last2d[1][1]
            angle = np.arctan2(self.last2d[1][1], self.last2d[1][0])
            res.pose.theta = angle
            res.found = True
        return res

    def _clear_filter(self, state):
        '''
        Reset filter and found state. This will ensure that the path marker
        is seen consistently before vision request returns true
        '''
        rospy.loginfo("MARKER LOST")
        self.found_count = 0
        self.found = False
        self.last3d = None
        self.filter.errorCovPre = 1. * np.eye(self.state_size, dtype=np.float32)
        if state is not None:
            self.found_count = 1
            state = np.array(state, dtype=np.float32)
            self.filter.statePost = state

    def _update_kf(self, (x, y, z, dy, dx)):
        '''
        Updates the path marker kalman filter using the pose estimation
        from the most recent frame. Also tracks time since last seen and how
        often is has been seen to set the boolean "found" for the vision request
        '''
        if self.last_found_time_3D is None: # First time found, set initial KF pose to this frame
            self._clear_filter((x, y, z, dy, dx))
            self.last_found_time_3D = self.image_sub.last_image_time
            return
        dt = (self.image_sub.last_image_time - self.last_found_time_3D).to_sec()
        self.last_found_time_3D = self.image_sub.last_image_time
        if dt <= 0 or dt > self.timeout_seconds:
            rospy.logwarn("Timed out since last saw marker, resetting")
            self._clear_filter((x, y, z, dy, dx))
            return

        self.found_count += 1
        measurement = 1.* np.array([x, y, z, dy, dx], dtype=np.float32)
        predict = self.filter.predict()
        estimated = self.filter.correct(measurement)
        if self.found_count > self.min_found_count:
              angle = np.arctan2(estimated[3], estimated[4])
              self.last3d = ((estimated[0], estimated[1], estimated[2]),
                              quaternion_from_euler(0.0, 0.0, angle))
              if not self.found:
                  rospy.loginfo("Marker Found")
              self.found = True

    def _get_pose_3D(self, p):
        i_points = np.array(p,dtype=np.float)
        # Use camera intrinsics and knowledge of marker's real demensions to get a pose estimate in camera frame
        _, rvec, tvec =  cv2.solvePnP(PathMarkerFinder.PATH_MARKER, i_points, self.cam.intrinsicMatrix(), np.zeros((5,1)))
        if tvec[2][0] < 0.3: # Sanity check on position estimate
            rospy.logwarn("Marker too close, must be wrong...")
            return False

        # Convert position estimate and 2d direction vector to messages to they can be transformed
        ps = PointStamped()
        ps.header.frame_id = self.cam.tfFrame()
        ps.header.stamp = self.image_sub.last_image_time
        ps.point = Point(*tvec)
        vec3 = Vector3Stamped()
        vec3.vector.x = self.last2d[1][0]
        vec3.vector.y = self.last2d[1][1]
        vec3.header.frame_id = self.cam.tfFrame()
        vec3.header.stamp = self.image_sub.last_image_time
        map_vec3 = None
        map_ps = None

        # Transform pose estimate to map framr
        try:
            self.tf_listener.waitForTransform('/map', ps.header.frame_id, ps.header.stamp, rospy.Duration(0.05))
            map_ps = self.tf_listener.transformPoint('/map', ps)
            map_vec3 = self.tf_listener.transformVector3('/map', vec3)
        except tf.Exception as err:
            rospy.logwarn("Could not transform {} to /map error={}".format(self.cam.tfFrame(),err))
            return False
        # Try to ensure vector always points the same way, so kf is not thrown off at some angles
        if map_vec3.vector.y < 0.0:
            map_vec3.vector.y = -map_vec3.vector.y
            map_vec3.vector.x = -map_vec3.vector.x
        measurement = (map_ps.point.x, map_ps.point.y, map_ps.point.z, map_vec3.vector.y, map_vec3.vector.x)

        # Update filter and found state with the pose estimate from this frame
        self._update_kf(measurement)

        if self.debug_ros:
            # Draw coordinate axis onto path marker using pose estimate to project
            refs, _ = cv2.projectPoints(PathMarkerFinder.REFERENCE_POINTS, rvec, tvec, self.cam.intrinsicMatrix(), np.zeros((5,1)))
            refs = np.array(refs, dtype=np.int)
            cv2.line(self.last_image, (refs[0][0][0], refs[0][0][1]), (refs[1][0][0], refs[1][0][1]), (0, 0, 255)) # X axis refs
            cv2.line(self.last_image, (refs[0][0][0], refs[0][0][1]), (refs[2][0][0], refs[2][0][1]), (0, 255, 0)) # Y axis ref
            cv2.line(self.last_image, (refs[0][0][0], refs[0][0][1]), (refs[3][0][0], refs[3][0][1]), (255, 0, 0)) # Z axis ref
            self._send_debug_marker()
        return True

    def _get_pose_2D(self, r):
        '''
        Given a sorted 4 sided polygon, stores the centriod and angle
        for the next service call to get 2dpose.
        returns False if dx of polygon is 0, otherwise True
        '''
        top_center = (r[1]+r[0])/2.0
        bot_center = (r[2]+r[3])/2.0
        vector = top_center - bot_center
        vector = vector / np.linalg.norm(vector)
        center = bot_center + (top_center - bot_center)/2.0
        vector = top_center - bot_center
        vector = vector/np.linalg.norm(vector)
        # Store last2d as a center pixel point and unit direction vector
        self.last2d = (center, vector)
        self.last_found_time_2D = self.image_sub.last_image_time
        return True

    def _is_valid_contour(self, contour):
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
            return False
        rect = self._sort_rect(polygon)
        if not self._get_pose_2D(rect):
            return False
        if self.do_3D:
            if not self._get_pose_3D(rect):
                return False
        return True

    def _get_edges(self):
        '''
        Proccesses latest image to find edges by:
        blurring and thresholding for highly saturated orangish objects
        then runs canny on threshold images and returns canny's edges
        '''
        blur = cv2.blur(self.last_image, (5,5))
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv, (0, self.thresh_saturation_low, 0), (self.thresh_hue_high, 255, 255))
        return cv2.Canny(thresh, self.canny_low, self.canny_low*self.canny_ratio)

    def _img_cb(self, img):
        if not self.enabled or self.cam == None:
            return
        self.last_image = img
        edges = self._get_edges()
        _, contours, _ = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        # Check if each contour is valid
        for idx, c in enumerate(contours):
            if self._is_valid_contour(c):
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
