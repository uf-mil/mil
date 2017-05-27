#!/usr/bin/env python
import numpy as np
from geometry_msgs.msg import Polygon
from mil_ros_tools.msg_helpers import rosmsg_to_numpy, numpy_to_polygon
import cv2

__author__ = "Kevin Allen"


class RectFinder(object):
    '''
    Keeps a model of a rectangle in meters, providing utility functions
    to find 2D and 3D pose estimations of rectangles matching this model
    found in a computer vision program.

    See github.com/uf-mil/SubjuGator/blob/master/perception/sub8_perception/nodes/path_marker_finder.py
    for an example of this used in a real time computer vision application.
    '''

    def __init__(self, length=1.0, width=1.0):
        '''
        Initializes the internal model of the rectangle.
        Also see RectFinder.from_polygon for creating this from a ROS message

        @param length the measurment of the longer side of the rectangle in meters
        @param width the measurment of the longer side of the rectangle in meters

        If width > length, the two will be reversed so that length is
        always the longer side.
        '''
        # Ensure length >= width
        self.width, self.length = tuple(np.sort([length, width]))
        self.model_3D = np.array([[self.length / 2.0, -self.width / 2.0, 0],
                                  [self.length / 2.0, self.width / 2.0, 0],
                                  [-self.length / 2.0, self.width / 2.0, 0],
                                  [-self.length / 2.0, -self.width / 2.0, 0]], dtype=np.float)
        # multiplied by 10000 for 0.1mm accuracy in integer image
        self.model_2D = np.array([[[0, 0]],
                                  [[self.width * 10000, 0]],
                                  [[self.width * 10000, self.length * 10000]],
                                  [[0, self.length * 10000]]], dtype=np.int)

    @classmethod
    def from_polygon(cls, polygon):
        '''
        Creates a RectFinder from a geometry_msgs/Polygon message.

        The length of the Rect becomes the range of x/y (whichever larger)

        The width of the Rect becomes the range of x/y (whichever shorter)
        '''
        assert isinstance(polygon, Polygon)
        arr = rosmsg_to_numpy(polygon.points)
        # If it contains just one point, treat the x/y and length/width
        if arr.shape[0] == 1:
            return cls(arr[:, 0][0], arr[:, 1][0])
        else:
            return cls(np.ptp(arr[:, 0]), np.ptp(arr[:, 1]))

    def to_polygon(self):
        '''
        Returns a geometry_msgs/Polygon representing the four corners
        of the rectangle where all z = 0
        '''
        return numpy_to_polygon(self.model_3D)

    @staticmethod
    def sort_corners(rect, debug_image=None):
        '''
        Given a contour of 4 points, returns the same 4 points sorted in a known way.
        Used so that indicies of contour line up to that in model for cv2.solvePnp

        @param debug_image if not None, puts a circle and text of the index in each corner

        p[0] = Top left corner         0--1     1--------2
        p[1] = Top right corner        |  |  or |        |
        p[2] = Bottom right corner     |  |     0--------3
        p[3] = Bottom left corner      3--2

        Credit to David Soto for this implementation.
        '''
        # Work with both (n, 1, 2) and (n, 2) shaped contour representations
        if rect.shape != (rect.shape[0], 2):
            M = cv2.moments(rect)
            rect = np.reshape(rect, (rect.shape[0], 2))
        else:
            temp = rect.reshape(rect.shape[0], 1, 2)
            M = cv2.moments(temp)
        centroid = np.array(
            (int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])))
        vectors = rect - centroid
        atan = np.arctan2(vectors[:, 1], vectors[:, 0])
        atan_indicies = np.argsort(atan)
        # Sort by arctan formed by vector from centroid to each point
        rect = rect[atan_indicies]
        # If rect is horizontal, correct indicies as noted above
        if np.linalg.norm(rect[0] - rect[1]) > np.linalg.norm(rect[0] - rect[3]):
            rect = rect[[3, 0, 1, 2]]
        # Print indicies onto image if debug_image is given
        if debug_image is not None:
            for i, pixel in enumerate(rect):
                center = (int(pixel[0]), int(pixel[1]))
                cv2.circle(debug_image, center, 5, (0, 255, 0), -1)
                cv2.putText(debug_image, str(i), center,
                            cv2.FONT_HERSHEY_SCRIPT_COMPLEX, 1, (0, 0, 255))
        return rect

    def verify_contour(self, contour):
        '''
        Returns a numerical comparison between a contour and the perfect
        model of the rectangle. A perfectly matched contour returns 0.0.

        Useful for filtering contours.
        '''
        return cv2.matchShapes(contour, self.model_2D, 3, 0.0)

    def get_corners(self, contour, epsilon_factor=0.02, debug_image=None):
        '''
        Attempts to find the 4 corners of a contour representing a quadrilateral.

        If the corners are found, it returns these 4 points sorted as described
        in sort_corners. These corners can be used for get_pose_2D and get_pose_3D

        If a 4 sided polygon cannot be approximated, returns None.

        @param epsilon_factor passed to OpenCV approx polygon function. May have to be tuned for your setup.
        @param debug_image if not None, will draw corners with text for indexes onto image

        TODO: instead of using one epsilon value, iterate until a 4 sided polygon is found
        '''
        epsilon = epsilon_factor * cv2.arcLength(contour, True)
        polygon = cv2.approxPolyDP(contour, epsilon, True)
        if len(polygon) != 4:
            return None
        return self.sort_corners(polygon, debug_image=debug_image)

    def get_pose_3D(self, corners, intrinsics=None, dist_coeffs=None, cam=None, rectified=False):
        '''
        Uses the model of the object, the corresponding pixels in the image, and camera
        intrinsics to estimate a 3D pose of the object.

        @param corners 4x2 numpy array from get_corners representing the 4 sorted corners in the image

        One of the two must be set:
        @param cam PinholeCameraModel object from ImageGeometry package
        @param rectified if cam is set, set True if corners were found in an
               already rectified image (image_rect_color topic)

        Or:
        @param instrinsics camera intrinisic matrix as numpy array
        @param dist_coeffs camera distortion coefficients as numpy array

        @return tuple (tvec, rvec) representing the translation and rotation vector
        between the camera and the object in meters/radians. Use cv2.Rodrigues to convert
        rvec to a 3x3 rotation matrix.
        '''
        corners = np.array(corners, dtype=np.float)
        # Use camera intrinsics and knowledge of marker's real demensions to
        # get a pose estimate in camera frame
        if cam is not None:
            intrinsics = cam.intrinsicMatrix()
            if rectified:
                dist_coeffs = np.zeros((5, 1))
            else:
                dist_coeffs = cam.distortionCoeffs()
        assert intrinsics is not None
        assert dist_coeffs is not None
        _, rvec, tvec = cv2.solvePnP(
            self.model_3D, corners, intrinsics, dist_coeffs)
        return (tvec, rvec)

    def get_pose_2D(self, corners):
        '''
        Finds the 2D center of the rectangle and a unit direction vector along the length
        of the rectangle in pixels

        @param corners 4x2 numpy array from get_corners representing the 4 sorted corners in the image

        @return a tuple (center, vecter)
        '''
        top_center = (corners[1] + corners[0]) / 2.0
        bot_center = (corners[2] + corners[3]) / 2.0
        vector = top_center - bot_center
        vector = vector / np.linalg.norm(vector)
        center = bot_center + (top_center - bot_center) / 2.0
        vector = top_center - bot_center
        vector = vector / np.linalg.norm(vector)
        # Store last2d as a center pixel point and unit direction vector
        return (center, vector)
