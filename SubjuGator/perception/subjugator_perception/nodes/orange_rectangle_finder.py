#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Point, PointStamped, Vector3Stamped
from image_geometry import PinholeCameraModel
from mil_msgs.srv import SetGeometry
from mil_ros_tools import (
    Image_Publisher,
    Image_Subscriber,
    numpy_to_point,
    numpy_to_quaternion,
)
from mil_vision_tools import RectFinder
from std_srvs.srv import SetBool, SetBoolResponse
from subjugator_msgs.srv import (
    VisionRequest,
    VisionRequest2D,
    VisionRequest2DResponse,
    VisionRequestResponse,
)
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

__author__ = "Kevin Allen"


# Ensure opencv3 or opencv4 is used, needed for KalmanFilter
assert cv2.__version__[0] in ["3", "4"]


class OrangeRectangleFinder:
    """
    Node which finds orange rectangular objects in image frame.
    This can be used for the path marker challenge and to detect
    the lid of the bins challenge. The node estimates the 2d and 3d
    position/orientation of this object and returns this estimate when service is called.

    Unit tests for this node is in test_path_marker.py

    Finding the marker works as follows:
    * blur image
    * threshold image mostly for highly saturated, orange/yellow/red objects
    * run canny edge detection on thresholded image
    * find contours on edge frame
    * filter contours to find those that may be contours by:
      * checking # of sides in appox polygon
      * checking ratio of length/width close to known model
    * estimates 3D pose using cv2.solvePnP with known object dimensions and camera model
    * Use translation vector from PnP and direction vector from 2d contour for pose
    * Transform this frames pose into /map frame
    * Plug this frames pose in /map into a kalman filter to reduce noise

    TODO: Allow for two objects to be identifed at once, both filtered through its own KF
    """

    # Coordinate axes for debugging image
    REFERENCE_POINTS = np.array(
        [[0, 0, 0], [0.3, 0, 0], [0, 0.3, 0], [0, 0, 0.3]],
        dtype=float,
    )

    def __init__(self):
        self.debug_gui = False
        self.enabled = False
        self.cam = None

        # Constants from launch config file
        self.debug_ros = rospy.get_param("~debug_ros", True)
        self.canny_low = rospy.get_param("~canny_low", 100)
        self.canny_ratio = rospy.get_param("~canny_ratio", 3.0)
        self.thresh_hue_high = rospy.get_param("~thresh_hue_high", 60)
        self.thresh_saturation_low = rospy.get_param("~thresh_satuation_low", 100)
        self.min_contour_area = rospy.get_param("~min_contour_area", 100)
        self.epsilon_range = rospy.get_param("~epsilon_range", (0.01, 0.1))
        self.epsilon_step = rospy.get_param("~epsilon_step", 0.01)
        self.shape_match_thresh = rospy.get_param("~shape_match_thresh", 0.4)
        self.min_found_count = rospy.get_param("~min_found_count", 10)
        self.timeout_seconds = rospy.get_param("~timeout_seconds", 2.0)
        # Default to scale model of path marker. Please use set_geometry service
        # to set to correct model of object.
        length = rospy.get_param("~length", 1.2192)
        width = rospy.get_param("~width", 0.1524)
        self.rect_model = RectFinder(length, width)
        self.do_3D = rospy.get_param("~do_3D", True)
        camera = rospy.get_param("~image_topic", "/camera/down/left/image_rect_color")

        self.tf_listener = tf.TransformListener()

        # Create kalman filter to track 3d position and direction vector for marker in /map frame
        self.state_size = 5  # X, Y, Z, DY, DX
        self.filter = cv2.KalmanFilter(self.state_size, self.state_size)
        self.filter.transitionMatrix = 1.0 * np.eye(self.state_size, dtype=np.float32)
        self.filter.measurementMatrix = 1.0 * np.eye(self.state_size, dtype=np.float32)
        self.filter.processNoiseCov = 1e-5 * np.eye(self.state_size, dtype=np.float32)
        self.filter.measurementNoiseCov = 1e-4 * np.eye(
            self.state_size,
            dtype=np.float32,
        )
        self.filter.errorCovPost = 1.0 * np.eye(self.state_size, dtype=np.float32)

        self.reset()
        self.service_set_geometry = rospy.Service(
            "~set_geometry",
            SetGeometry,
            self._set_geometry_cb,
        )
        if self.debug_ros:
            self.debug_pub = Image_Publisher("~debug_image")
            self.markerPub = rospy.Publisher("~marker", Marker, queue_size=10)
        self.service2D = rospy.Service("~2D", VisionRequest2D, self._vision_cb_2D)
        if self.do_3D:
            self.service3D = rospy.Service("~pose", VisionRequest, self._vision_cb_3D)
        self.toggle = rospy.Service("~enable", SetBool, self._enable_cb)

        self.image_sub = Image_Subscriber(camera, self._img_cb)
        self.camera_info = self.image_sub.wait_for_camera_info()
        assert self.camera_info is not None
        self.cam = PinholeCameraModel()
        self.cam.fromCameraInfo(self.camera_info)

    def _set_geometry_cb(self, req):
        self.rect_model = RectFinder.from_polygon(req.model)
        self.reset()
        rospy.loginfo(
            "Resetting rectangle model to LENGTH=%f, WIDTH=%f",
            self.rect_model.length,
            self.rect_model.width,
        )
        return {"success": True}

    def _send_debug_marker(self):
        """
        Sends a rviz marker in the camera frame with the estimated pose of the object.
        This marker is a scaled cube with the dimensions of the model.
        Only called if debug_ros param == True
        """
        if self.last3d is None or not self.found:
            return
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.last_found_time_3D
        m.ns = "orange_rectangle"
        m.id = 0
        m.type = 1
        m.action = 0
        # Real dimensions of path marker
        m.scale.x = self.rect_model.length
        m.scale.y = self.rect_model.width
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
            self.reset()
            self.tf_listener.clear()
        self.enabled = x.data
        return SetBoolResponse(success=True)

    def _vision_cb_3D(self, req):
        res = VisionRequestResponse()
        if self.last_found_time_3D is None or self.image_sub.last_image_time is None:
            res.found = False
            return res
        dt = (self.image_sub.last_image_time - self.last_found_time_3D).to_sec()
        if (
            dt < 0
            or dt > self.timeout_seconds
            or (self.last3d is None or not self.enabled)
        ):
            res.found = False
        else:
            res.pose.header.frame_id = "map"
            res.pose.header.stamp = self.last_found_time_3D
            res.pose.pose.position = numpy_to_point(self.last3d[0])
            res.pose.pose.orientation = numpy_to_quaternion(self.last3d[1])
            res.found = True
        return res

    def _vision_cb_2D(self, req):
        res = VisionRequest2DResponse()
        if self.last2d is None or not self.enabled:
            res.found = False
        else:
            res.header.frame_id = self.cam.tfFrame()
            res.header.stamp = self.last_found_time_2D
            res.pose.x = self.last2d[0][0]
            res.pose.y = self.last2d[0][1]
            res.camera_info = self.camera_info
            res.max_x = self.camera_info.width
            res.max_y = self.camera_info.height
            if self.last2d[1][0] < 0:
                self.last2d[1][0] = -self.last2d[1][0]
                self.last2d[1][1] = -self.last2d[1][1]
            angle = np.arctan2(self.last2d[1][1], self.last2d[1][0])
            res.pose.theta = angle
            res.found = True
        return res

    def reset(self):
        self.last_found_time_2D = None
        self.last_found_time_3D = None
        self.last2d = None
        self.last3d = None
        self._clear_filter(None)

    def _clear_filter(self, state):
        """
        Reset filter and found state. This will ensure that the object
        is seen consistently before vision request returns true
        """
        rospy.loginfo("MARKER LOST")
        self.found_count = 0
        self.found = False
        self.last3d = None
        self.filter.errorCovPre = 1.0 * np.eye(self.state_size, dtype=np.float32)
        if state is not None:
            self.found_count = 1
            state = np.array(state, dtype=np.float32)
            self.filter.statePost = state

    def _update_kf(self, kf_args):
        """
        Updates the kalman filter using the pose estimation
        from the most recent frame. Also tracks time since last seen and how
        often is has been seen to set the boolean "found" for the vision request
        """
        (x, y, z, dy, dx) = kf_args
        if (
            self.last_found_time_3D is None
        ):  # First time found, set initial KF pose to this frame
            self._clear_filter((x, y, z, dy, dx))
            self.last_found_time_3D = self.image_sub.last_image_time
            return
        dt = (self.image_sub.last_image_time - self.last_found_time_3D).to_sec()
        self.last_found_time_3D = self.image_sub.last_image_time
        if dt < 0 or dt > self.timeout_seconds:
            rospy.logwarn(f"Timed out since last saw marker, resetting. DT={dt}")
            self._clear_filter((x, y, z, dy, dx))
            return

        self.found_count += 1
        measurement = 1.0 * np.array([x, y, z, dy, dx], dtype=np.float32)
        self.filter.predict()
        estimated = self.filter.correct(measurement)
        if self.found_count > self.min_found_count:
            angle = np.arctan2(estimated[3], estimated[4])
            self.last3d = (
                (estimated[0], estimated[1], estimated[2]),
                quaternion_from_euler(0.0, 0.0, angle),
            )
            if not self.found:
                rospy.loginfo("Marker Found")
            self.found = True

    def _get_pose_3D(self, corners):
        tvec, rvec = self.rect_model.get_pose_3D(corners, cam=self.cam, rectified=True)
        if tvec[2][0] < 0.3:  # Sanity check on position estimate
            rospy.logwarn("Marker too close, must be wrong...")
            return False
        rmat, _ = cv2.Rodrigues(rvec)
        vec = rmat[:, 0]

        # Convert position estimate and 2d direction vector to messages to they can be transformed
        ps = PointStamped()
        ps.header.frame_id = self.cam.tfFrame()
        ps.header.stamp = self.image_sub.last_image_time
        ps.point = Point(*tvec)
        vec3 = Vector3Stamped()
        vec3.vector.x = vec[0]
        vec3.vector.y = vec[1]
        vec3.header.frame_id = self.cam.tfFrame()
        vec3.header.stamp = self.image_sub.last_image_time
        map_vec3 = None
        map_ps = None

        # Transform pose estimate to map frame
        try:
            self.tf_listener.waitForTransform(
                "map",
                ps.header.frame_id,
                ps.header.stamp,
                rospy.Duration(0.1),
            )
            map_ps = self.tf_listener.transformPoint("map", ps)
            map_vec3 = self.tf_listener.transformVector3("map", vec3)
        except tf.Exception as err:
            rospy.logwarn(
                f"Could not transform {self.cam.tfFrame()} to map error={err}",
            )
            return False
        # Try to ensure vector always points the same way, so kf is not thrown off at some angles
        if map_vec3.vector.y < 0.0:
            map_vec3.vector.y = -map_vec3.vector.y
            map_vec3.vector.x = -map_vec3.vector.x
        measurement = (
            map_ps.point.x,
            map_ps.point.y,
            map_ps.point.z,
            map_vec3.vector.y,
            map_vec3.vector.x,
        )

        # Update filter and found state with the pose estimate from this frame
        self._update_kf(measurement)

        if self.debug_ros:
            # Draw coordinate axis onto object using pose estimate to project
            refs, _ = cv2.projectPoints(
                self.REFERENCE_POINTS,
                rvec,
                tvec,
                self.cam.intrinsicMatrix(),
                np.zeros((5, 1)),
            )
            refs = np.array(refs, dtype=np.int)
            cv2.line(
                self.last_image,
                (refs[0][0][0], refs[0][0][1]),
                (refs[1][0][0], refs[1][0][1]),
                (0, 0, 255),
            )  # X axis refs
            cv2.line(
                self.last_image,
                (refs[0][0][0], refs[0][0][1]),
                (refs[2][0][0], refs[2][0][1]),
                (0, 255, 0),
            )  # Y axis ref
            cv2.line(
                self.last_image,
                (refs[0][0][0], refs[0][0][1]),
                (refs[3][0][0], refs[3][0][1]),
                (255, 0, 0),
            )  # Z axis ref
            self._send_debug_marker()
        return True

    def _is_valid_contour(self, contour):
        """
        Does various tests to filter out contours that are clearly not
        a valid orange rectangle.
        * run approx polygon, check that sides == 4
        * find ratio of length to width, check close to known ratio IRL
        """
        if cv2.contourArea(contour) < self.min_contour_area:
            return False
        match = self.rect_model.verify_contour(contour)
        if match > self.shape_match_thresh:
            return False
        # Checks that contour is 4 sided
        corners = self.rect_model.get_corners(
            contour,
            debug_image=self.last_image,
            epsilon_range=self.epsilon_range,
            epsilon_step=self.epsilon_step,
        )
        if corners is None:
            return False
        self.last2d = self.rect_model.get_pose_2D(corners)
        self.last_found_time_2D = self.image_sub.last_image_time
        if self.do_3D and not self._get_pose_3D(corners):
            return False
        return True

    def _get_edges(self):
        """
        Processes latest image to find edges by:
        blurring and thresholding for highly saturated orangish objects
        then runs canny on threshold images and returns canny's edges
        """
        blur = cv2.blur(self.last_image, (5, 5))
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        thresh = cv2.inRange(
            hsv,
            (0, self.thresh_saturation_low, 0),
            (self.thresh_hue_high, 255, 255),
        )
        return cv2.Canny(thresh, self.canny_low, self.canny_low * self.canny_ratio)

    def _img_cb(self, img):
        if not self.enabled or self.cam is None:
            return
        self.last_image = img
        edges = self._get_edges()
        _, contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Check if each contour is valid
        for idx, c in enumerate(contours):
            if self._is_valid_contour(c):
                if self.debug_ros:
                    cv2.drawContours(self.last_image, contours, idx, (0, 255, 0), 3)
                break
            else:
                if self.debug_ros:
                    cv2.drawContours(self.last_image, contours, idx, (255, 0, 0), 3)
        if self.debug_ros:
            self.debug_pub.publish(self.last_image)
        if self.debug_gui:
            cv2.imshow("debug", self.last_image)
            cv2.waitKey(5)


if __name__ == "__main__":
    rospy.init_node("orange_rectangle_finder")
    OrangeRectangleFinder()
    rospy.spin()
