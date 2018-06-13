from __future__ import print_function

import sys
import rospy
import tf
import mil_ros_tools
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from sub8_vision_tools import MultiObservation
from image_geometry import PinholeCameraModel
from sub8_msgs.srv import VisionRequest2DResponse, VisionRequest2D, VisionRequest, VisionRequestResponse
from mil_ros_tools import Image_Subscriber, Image_Publisher, rosmsg_to_numpy
from collections import deque
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge, CvBridgeError
'''
BGR Color space constants for thresholding. We are looking for red so
the third value should have the largest range.
'''
LOWER = [0, 0, 40]
UPPER = [50, 50, 250]

# Length threshold for contours. Contours smaller than this size are ignored.
SIZE = 100

# How many pixels off from center are acceptable
CENTER_X_THRESH = 15
CENTER_Y_THRESH = 15


class torp_vision:

    def __init__(self):
        self.tf_listener = tf.TransformListener()
        self._observations = deque()
        self.status = ''
        self.est = None
        self.draw_colors = (1.0, 0.0, 0.0, 1.0)
        self.cv_colors = (0, 0, 255, 0)
        self.visual_id = 0
        self._pose_pairs = deque()
        self._times = deque()
        self.timeout = rospy.Duration()

        self.last_image = None
        self.min_observations = 10
        # self.rviz = None
        self.mem = np.zeros((2, 10))
        self.bridge = CvBridge()
        self.debug_ros = True
        # For legit testing "/camera/front/right/image_color"
        self.enabled = False

        self.image_sub = Image_Subscriber(
            "/camera/front/right/image_rect_color", self.callback)

        self.last_image_time = self.image_sub.last_image_time
        self.camera_info = self.image_sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        self.frame_id = self.camera_model.tfFrame()

        rospy.Service('~enable', SetBool, self.toggle_search)
        self.multi_obs = MultiObservation(self.camera_model)
        rospy.Service('~pose', VisionRequest, self.request_board3d)
        self.image_pub = Image_Publisher(
            "torp_vision/debug")
        self.point_pub = rospy.Publisher(
            "torp_vision/points", Point, queue_size=1)

    def toggle_search(self, srv):
        '''
        Callback for standard ~enable service. If true, start
        looking at frames for buoys.
        '''
        if srv.data:
            rospy.loginfo("TARGET ACQUISITION: enabled")
            self.enabled = True
        else:
            rospy.loginfo("TARGET ACQUISITION: disabled")
            self.enabled = False

        return SetBoolResponse(success=True)

    def request_board3d(self, srv):
        '''
        Callback for 3D vision request. Uses recent observations of target board
        specified in target_name to attempt a least-squares position estimate.
        Ignoring orientation of board.
        '''
        if not self.enabled:
            return VisionRequestResponse(found=False)
        # buoy = self.buoys[srv.target_name]
        if self.est is None:
            return VisionRequestResponse(found=False)
        return VisionRequestResponse(
            pose=PoseStamped(
                header=Header(stamp=self.last_image_time, frame_id='/map'),
                pose=Pose(
                    position=Point(*self.est)
                )
            ),
            found=True
        )

    def clear_old_observations(self):
        time = rospy.Time.now()
        i = 0
        while i < len(self._times):
            if time - self._times[i] > self.timeout:
                self._times.popleft()
                self._observations.popleft()
                self._pose_pairs.popleft()
            else:
                i += 1

    def size(self):
        return len(self._observations)

    def add_observation(self, obs, pose_pair, time):
        self.clear_old_observations()
        if self.size() == 0 or np.linalg.norm(self._pose_pairs[-1][0] - pose_pair[0]) > self.min_trans:
            self._observations.append(obs)
            self._pose_pairs.append(pose_pair)
            self._times.append(time)

    def get_observations_and_pose_pairs(self):
        self.clear_old_observations()
        return (self._observations, self._pose_pairs)

    def detect(self, c):
        # initialize the shape name and approximate the contour
        target = "unidentified"
        peri = cv2.arcLength(c, True)
        if peri < SIZE:
            return target
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        if len(approx) == 5 or len(approx) == 4:
            # compute the bounding box of the contour and use the
            # bounding box to compute the aspect ratio
            # (x, y, w, h) = cv2.boundingRect(approx)
            # ar = w / float(h)
            target = "verified shooty hole"

        elif len(approx) == 3:
            # (x, y, w, h) = cv2.boundingRect(approx)
            # ar = w / float(h)
            target = "partial shooty hole"
        return target

    def callback(self, data):
        # try:
        #     cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # except CvBridgeError as e:
        #     print(e)
        self.last_image_time = self.image_sub.last_image_time
        cv_image = data
        height, width, channels = cv_image.shape
        # print(height)
        # print(width)
        # CLAHE (Contrast Limited Adaptive Histogram Equalization)
        # clahe = cv2.createCLAHE(clipLimit=1., tileGridSize=(4, 4))

        # # convert from BGR to LAB color space
        # lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)
        # l, a, b = cv2.split(lab)  # split on 3 different channels

        # l2 = clahe.apply(l)  # apply CLAHE to the L-channel

        # lab = cv2.merge((l2, a, b))  # merge channels
        # cv_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        # create NumPy arrays from the boundaries
        lower = np.array(LOWER, dtype="uint8")
        upper = np.array(UPPER, dtype="uint8")

        # Generate a mask based on the constants.
        mask = cv2.inRange(cv_image, lower, upper)
        # Remove anything not within the bounds of our mask
        output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # Resize to emphasize shapes
        resized = cv2.resize(output, (300, 225))
        ratio = output.shape[0] / float(resized.shape[0])
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
        # Blur image so our contours can better find the full shape.
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # Compute contours
        cnts = cv2.findContours(blurred.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)

        cnts = cnts[1]

        peri_max = 0
        max_x = 0
        max_y = 0

        for c in cnts:
            # compute the center of the contour, then detect the name of the
            # shape using only the contour
            M = cv2.moments(c)
            if M["m00"] == 0:
                M["m00"] = .000001
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            shape = self.detect(c)

            # multiply the contour (x, y)-coordinates by the resize ratio,
            # then draw the contours and the name of the shape on the image

            c = c.astype("float")
            c *= ratio
            c = c.astype("int")
            if shape == "verified shooty hole" or shape == "partial shooty hole" or shape == "unidentified":
                cv2.drawContours(output, [c], -1, (0, 255, 0), 2)

                cv2.putText(output, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 2)
                peri = cv2.arcLength(c, True)
                if peri > peri_max:
                    peri_max = peri
                    max_x = cX
                    max_y = cY

        m = Point()
        m.x = max_x
        m.y = max_y
        temp1 = max_y - (height / 2)
        temp2 = max_x - (width / 2)
        # print("temp 1: ", temp1)
        # print("temp 2: ", temp2)
        try:
            self.tf_listener.waitForTransform(
                '/map', self.frame_id, self.last_image_time, rospy.Duration(0.2))
        except tf.Exception as e:
            rospy.logwarn("Could not transform camera to map: {}".format(e))
            return False

        # if not self.sanity_check(center, self.last_image_time):
        #     self.status = 'failed sanity check'
        #     return False

        (t, rot_q) = self.tf_listener.lookupTransform(
            '/map', self.frame_id, self.last_image_time)
        R = mil_ros_tools.geometry_helpers.quaternion_matrix(rot_q)
        center = np.array([max_x, max_y])
        self.add_observation(center, (np.array(t), R), self.last_image_time)

        observations, pose_pairs = self.get_observations_and_pose_pairs()
        if len(observations) > self.min_observations:
            self.est = self.multi_obs.lst_sqr_intersection(
                observations, pose_pairs)
            self.status = 'Pose found'
            # if self.debug_ros:
            # self.rviz.draw_sphere(self.est, color=self.draw_colors,
            # scaling = (0.2286, 0.2286, 0.2286),
            # frame = '/map', _id = self.visual_id)
        else:
            self.status = '{} observations'.format(len(observations))
        # Center and Radius
        # return center, radius
        '''
        This is a crime against ROS Messages but if it works...... it works. I apologize in advance.
        '''
        m.z = 0
        if (abs(temp2) > (CENTER_X_THRESH)):
            if (temp1 < 0):
                m.z -= 5
            else:
                m.z += 5
        if (abs(temp1) > (CENTER_Y_THRESH)):
            if (temp2 < 0):
                m.z -= 1
            else:
                m.z += 1
        if(m.x == 0) or (m.y == 0):
            m.z = 0
        '''
        This gives the following possibilities:
        # Y ### Range of Values: (-5, 5)
        m.z = -5 --> Only X thresh is off and sub is too low.
        m.z = 5 --> Only X thresh is off and sub is too high.

        # BOTH ### Range of Values: (4, 6, -4, -6)
        m.z = 4 --> Both are off, too high, too far right.
        m.z = 6 --> Both are off, too high, too far left.
        m.z = -4 --> Both are off, too low, too far left.
        m.z = -6 --> Both are off, too low, too far right.

        # X ### Range of Values(-1, 1)
        m.z = -1 --> Only Y is off, too far right.
        m.z = 1 --> Only Y is off, too far left.
        '''

        try:
            self.point_pub.publish(m)
            self.image_pub.publish(output)
            return center
            # print("success")
        except CvBridgeError as e:
            print(e)


def main(args):
    rospy.init_node('torp_vision', anonymous=True)
    torp_vision()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
