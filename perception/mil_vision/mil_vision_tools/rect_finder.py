#!/usr/bin/env python
import numpy as np
from geometry_msgs.msg import Polygon
from mil_ros_tools.msg_helpers import rosmsg_to_numpy, numpy_to_polygon
import cv2

__author__ = "Kevin Allen"

class RectFinder(object):
    def __init__(self, length=1.0, width=1.0):
        '''
        p[0] = Top left corner of marker         0-W-1
        p[1] = Top right corner of marker        |   |
                                                 L   L
        p[2] = Bottom right corner of marker     |   |
        p[3] = Bottom left cornet of marker      3-W-2
        '''
        # Ensure length >= width
        self.width, self.length = tuple(np.sort([length, width]))
        self.model_3D = np.array([[self.length/2.0,  -self.width/2.0, 0],
                                  [self.length/2.0,   self.width/2.0,  0],
                                  [-self.length/2.0,  self.width/2.0, 0],
                                  [-self.length/2.0, -self.width/2.0, 0]], dtype=np.float)
        # multiplied by 10000 for 0.1mm accuracy in integer image
        self.model_2D = np.array([[[0, 0]],
                               [[self.width*10000, 0]],
                               [[self.width*10000, self.length*10000]],
                               [[0, self.length*10000]]], dtype=np.int)

    @classmethod
    def from_polygon(cls, polygon):
        assert isinstance(polygon, Polygon)
        arr = rosmsg_to_numpy(polygon.points)
        if arr.shape[0] == 1:
            return cls(arr[:,0][0], arr[:,1][0])
        else:
            return cls(np.ptp(arr[:,0]), np.ptp(arr[:,1]))

    def to_polygon(self):
        return numpy_to_polygon(self.model_3D)

    @staticmethod
    def sort_corners(rect, debug_image=None):
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
        if rect.shape != (rect.shape[0], 2):
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
        if debug_image is not None:
            for i, pixel in enumerate(rect):
                center = (int(pixel[0]), int(pixel[1]))
                cv2.circle(debug_image, center, 5, (0, 255, 0), -1)
                cv2.putText(debug_image, str(i), center, cv2.FONT_HERSHEY_SCRIPT_COMPLEX,1, (0,0,255))
        return rect

    def verify_contour(self, contour):
        '''
        Does various tests to filter out contours that are clearly not
        a valid path marker.
        * run approx polygon, check that sides == 4
        * find ratio of length to width, check close to known ratio IRL
        '''
        return cv2.matchShapes(contour, self.model_2D, 3, 0.0)

    def get_corners(self, contour, epsilon_factor=0.1, debug_image=None):
        epsilon = epsilon_factor*cv2.arcLength(contour,True)
        polygon = cv2.approxPolyDP(contour,epsilon,True)
        if len(polygon) != 4:
            return None
        return self.sort_corners(polygon, debug_image=debug_image)

    def get_pose_3D(self, corners, intrinsics=None, dist_coeffs=None, cam=None, rectified=False):
        corners = np.array(corners,dtype=np.float)
        # Use camera intrinsics and knowledge of marker's real demensions to get a pose estimate in camera frame
        if cam is not None:
            intrinsics = cam.intrinsicMatrix()
            if rectified:
                dist_coeffs = np.zeros((5,1))
            else:
                dist_coeffs = distortionCoeffs()
        assert intrinsics is not None
        assert dist_coeffs is not None
        _, rvec, tvec =  cv2.solvePnP(self.model_3D, corners, intrinsics, dist_coeffs)
        return (tvec, rvec)

    def get_pose_2D(self, corners):
        '''
        Given a sorted 4 sided polygon, stores the centroid and angle
        for the next service call to get 2dpose.
        '''
        top_center = (corners[1]+corners[0])/2.0
        bot_center = (corners[2]+corners[3])/2.0
        vector = top_center - bot_center
        vector = vector / np.linalg.norm(vector)
        center = bot_center + (top_center - bot_center)/2.0
        vector = top_center - bot_center
        vector = vector/np.linalg.norm(vector)
        # Store last2d as a center pixel point and unit direction vector
        return (center, vector)
