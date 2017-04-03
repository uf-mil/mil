#!/usr/bin/env python
import cv2
import numpy as np
import sys
import rospy
import rospkg
from tf.transformations import quaternion_from_euler
import os
from std_msgs.msg import Header
from std_srvs.srv import SetBool, SetBoolResponse
from sub8_msgs.srv import VisionRequestResponse, VisionRequest, VisionRequest2D, VisionRequest2DResponse, VisionRequest, VisionRequestResponse
from geometry_msgs.msg import PoseStamped
from sub8_ros_tools import numpy_quat_pair_to_pose, numpy_to_point, numpy_matrix_to_quaternion, numpy_to_quaternion
from sub8_ros_tools import Image_Publisher, Image_Subscriber
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker

rospack = rospkg.RosPack()

'''
Game plan:
* grayscale
* binary threshold (bright stuf??)
* blur (bilatteral)
* canny (?)
* contours
* polygon approx
* filters
  - polygon approx is 4 sided
  - area (somewhat large)
  - rectangular demensions (in rulebook, only check roughly)
* cache response pose
  - x,y from centroid (sub8 tools)
  - middle of two skinny sides with numpy
  - theta with atan of two skinny sides
  - 

* buffer / tracking / kalman filter
  - so one bad frame doesn't fuck us
* publish markers for debug
'''

class PathMarkerFinder():
    # Model of math marker, centered around 0 in meters
    #~ PATH_MARKER = np.array([[-0.0762, -0.6096,  0],
                            #~ [0.0762,  -0.6096,  0],
                            #~ [0.0762,  0.6096, 0],
                            #~ [-0.0762, 0.6096, 0]], dtype=np.float)
    PATH_MARKER = np.array([[0.6096,  -0.0762, 0],
                            [0.6096,  0.0762,  0],
                            [-0.6096,  0.0762, 0],
                            [-0.6096, -0.0762, 0]], dtype=np.float)
    REFERENCE_POINTS = np.array([[0, 0, 0],
                                 [0.3, 0, 0],
                                 [0, 0.3, 0],
                                 [0, 0, 0.3]], dtype=np.float)
    def __init__(self):
        self.debug_ros = True
        self.debug_gui = False
        self.last2d = None
        self.last3d = None
        self.canny_low = 100
        self.canny_ratio = 3.0
        camera = rospy.get_param("camera", "/down_camera/image_rect_color")
        self.service2D = rospy.Service('/vision/path_marker/2D', VisionRequest2D, self.cb_2d)
        self.service3D = rospy.Service('/vision/path_marker/pose', VisionRequest, self.cb_3d)
        self.toggle = rospy.Service('/vision/path_marker/enable', SetBool, self.enable_cb)
        self.enabled = False
        self.image_sub = Image_Subscriber(camera, self.img_cb)
        self.camera_info = self.image_sub.wait_for_camera_info() 
        self.cam = PinholeCameraModel()
        self.cam.fromCameraInfo(self.camera_info)
        self.last_im = None
        self.last_found_time = None
        if self.debug_ros:
            self.debug_pub = Image_Publisher("debug_image")
            self.markerPub = rospy.Publisher('path_marker_visualization', Marker, queue_size=10)
        self.enabled = True

    def sendDebugMarker(self):
        m = Marker()
        m.header.frame_id = self.image_sub.camera_info.header.frame_id
        m.header.stamp = self.last_found_time
        m.ns = "path_markers"
        m.id = 0
        m.type = 1 # sphere
        m.action = 0
        m.pose.position = numpy_to_point(self.last3d[0])
        m.pose.orientation = numpy_to_quaternion(self.last3d[1])
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
        self.enabled = x.data

    def sortRect(self, rect):
        # Sorts four corners in a consistent way
        def sort_contour_y(c):
            return c[0][1]
        def sort_contour_x(c):
            return c[0][0]
        sorted_y = np.array(sorted(rect, key=sort_contour_y))
        sorted_x = np.array(sorted(rect, key=sort_contour_x))
        
        horizontal = False
        if (np.linalg.norm(sorted_y[0] - sorted_y[1]) > np.linalg.norm(sorted_y[0] - sorted_y[2]) or
                np.linalg.norm(sorted_y[0] - sorted_y[1]) > np.linalg.norm(sorted_y[0] - sorted_y[3])):
            horizontal = True

        sorted_rect = None
        if horizontal:
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
        return sorted_rect

    def cb_3d(self, req):
        res = VisionRequestResponse()
        res.pose.header.frame_id = self.image_sub.camera_info.header.frame_id
        res.pose.header.stamp = self.last_found_time
        if (self.last2d == None):
            res.found = False
        else:
            res.pose.pose.position = numpy_to_point(self.last3d[0])
            res.pose.pose.orientation = numpy_to_quaternion(self.last3d[1])
            res.found = True
        return res
    
    def cb_2d(self, req):
        res = VisionRequest2DResponse()
        res.header.frame_id = self.image_sub.camera_info.header.frame_id
        res.header.stamp = self.last_found_time
        if (self.last3d == None):
            res.found = False
        else:
            res.pose.x = self.last2d[0][0]
            res.pose.y = self.last2d[0][1]
            res.pose.theta = self.last2d[1]
            res.found = True
        return res

    def get_3d_pose(self, p):
        i_points = np.array((p[0][0], p[1][0], p[2][0], p[3][0]),dtype=np.float)
        retval, rvec, tvec =  cv2.solvePnP(PathMarkerFinder.PATH_MARKER, i_points, self.cam.intrinsicMatrix(), np.zeros((5,1)))
        if tvec[2] < 0.3 :
            return False
        self.last3d = (tvec.copy(), quaternion_from_euler(0.0, 0.0, self.last2d[1]))
        if self.debug_ros:
            refs, _ = cv2.projectPoints(PathMarkerFinder.REFERENCE_POINTS, rvec, tvec, self.cam.intrinsicMatrix(), np.zeros((5,1)))
            refs = np.array(refs, dtype=np.int)
            cv2.line(self.last_image, (refs[0][0][0], refs[0][0][1]), (refs[1][0][0], refs[1][0][1]), (0, 0, 255)) # X axis refs
            cv2.line(self.last_image, (refs[0][0][0], refs[0][0][1]), (refs[2][0][0], refs[2][0][1]), (0, 255, 0)) # Y axis ref
            cv2.line(self.last_image, (refs[0][0][0], refs[0][0][1]), (refs[3][0][0], refs[3][0][1]), (255, 0, 0)) # Z axis ref
        return True
    
    def get_2d_pose(self, r):
        top_center = r[0][0] + (r[1][0]-r[0][0])/2.0
        bot_center = r[2][0] + (r[3][0]-r[2][0])/2.0
        center = bot_center + (top_center - bot_center)/2.0
        angle = np.arctan( (top_center[1]-bot_center[1]) / (top_center[0] - bot_center[0]) )
        self.last2d = (center, angle)
        return True

    def valid_contour(self, contour):
        polygon = cv2.approxPolyDP(contour, 10, True)
        if len(polygon) != 4:
            return False
        rect = self.sortRect(polygon)
        for idx, p in enumerate(rect):
            cv2.putText(self.last_image, str(idx), (p[0][0], p[0][1]), cv2.FONT_HERSHEY_SCRIPT_COMPLEX,1, (0,0,255))
        length_width_ratio = np.linalg.norm(rect[0][0]-rect[3][0])/np.linalg.norm(rect[0][0]-rect[1][0])
        if abs(length_width_ratio-8.0)/8.0 > 0.2:
            return False
        if not self.get_2d_pose(rect):
            return False
        if not self.get_3d_pose(rect):
            return False
        return True

    def get_edges(self):
        blur = cv2.blur(self.last_image, (5,5))
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(hsv, (0, 100, 0), (60, 255, 255))
        return cv2.Canny(thresh, self.canny_low, self.canny_low*self.canny_ratio)

    def img_cb(self, img):
        if not self.enabled:
            return
        self.last_image = img
        edges = self.get_edges()
        _, contours, hierarchy = cv2.findContours(edges,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours_img = img.copy()
        for idx, c in enumerate(contours):
            area = cv2.contourArea(c)
            if area > 100:
                if self.valid_contour(c):
                    rospy.loginfo("Found path marker")
                    self.last_found_time = self.image_sub.last_image_time
                    if self.debug_ros:
                        self.sendDebugMarker()
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
