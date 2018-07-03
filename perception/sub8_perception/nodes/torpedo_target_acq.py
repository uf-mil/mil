#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import tf
import mil_ros_tools
import cv2
import numpy as np
from std_msgs.msg import Header
from collections import deque
from cv_bridge import CvBridgeError
from sub8_vision_tools import MultiObservation
from image_geometry import PinholeCameraModel
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped, Pose, Point
from sub8_msgs.srv import VisionRequest, VisionRequestResponse
from mil_ros_tools import Image_Subscriber, Image_Publisher
'''
Perception component of the Torpedo Board Challenge. Utilizes code from
the pyimagesearch blog post on color thresholding and shape detection
as well as code from the buoy_finder mission of previous years.
'''


class torp_vision:
    def __init__(self):

        # Pull constants from config file
        self.lower = rospy.get_param('~lower_color_threshold', [0, 0, 60])
        self.upper = rospy.get_param('~higher_color_threshold', [60, 60, 250])
        self.min_contour_area = rospy.get_param('~min_contour_area', .001)
        self.min_trans = rospy.get_param('~min_trans', .05)
        self.max_velocity = rospy.get_param('~max_velocity', 1)
        self.timeout = rospy.Duration(rospy.get_param('~timeout_seconds'))
        self.min_observations = rospy.get_param('~min_observations', 8)
        self.camera = rospy.get_param('~camera_tropic',
                                      '/camera/front/right/image_rect_color')

        # Instantiate remaining variables and objects
        self._observations = deque()
        self._pose_pairs = deque()
        self._times = deque()
        self.last_image_time = None
        self.last_image = None
        self.tf_listener = tf.TransformListener()
        self.status = ''
        self.est = None
        self.visual_id = 0
        self.enabled = False

        # Image Subscriber and Camera Information

        self.image_sub = Image_Subscriber(self.camera, self.image_cb)
        self.camera_info = self.image_sub.wait_for_camera_info()
        '''
        These variables store the camera information required to perform
        the transformations on the coordinates to move from the subs
        perspective to our global map perspective. This information is
        also necessary to perform the least squares intersection which
        will find the 3D coordinates of the torpedo board based on 2D
        observations from the Camera. More info on this can be found in
        sub8_vision_tools.
        '''

        self.camera_info = self.image_sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        self.frame_id = self.camera_model.tfFrame()

        # Ros Services so mission can be toggled and info requested
        rospy.Service('~enable', SetBool, self.toggle_search)
        self.multi_obs = MultiObservation(self.camera_model)
        rospy.Service('~pose', VisionRequest, self.request_board3d)
        self.image_pub = Image_Publisher("torp_vision/debug")
        self.point_pub = rospy.Publisher(
            "torp_vision/points", Point, queue_size=1)

        # Debug
        self.debug = rospy.get_param('~debug', True)

    def image_cb(self, image):
        '''
        Run each time an image comes in from ROS. If enabled,
        attempt to find the torpedo board.
        '''
        if not self.enabled:
            return

        self.last_image = image

        if self.last_image_time is not None and \
                self.image_sub.last_image_time < self.last_image_time:
            # Clear tf buffer if time went backwards (nice for playing bags in
            # loop)
            self.tf_listener.clear()

        self.last_image_time = self.image_sub.last_image_time
        self.acquire_targets(image)

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
        Callback for 3D vision request. Uses recent observations of target
        board  specified in target_name to attempt a least-squares position
        estimate. Ignoring orientation of board.
        '''
        if not self.enabled:
            return VisionRequestResponse(found=False)
        # buoy = self.buoys[srv.target_name]
        if self.est is None:
            return VisionRequestResponse(found=False)
        return VisionRequestResponse(
            pose=PoseStamped(
                header=Header(stamp=self.last_image_time, frame_id='/map'),
                pose=Pose(position=Point(*self.est))),
            found=True)

    def clear_old_observations(self):
        # Observations older than two seconds are discarded.
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
        if self.size() == 0 or np.linalg.norm(
                self._pose_pairs[-1][0] - pose_pair[0]) > self.min_trans:
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

        if peri < self.min_contour_area:
            return target
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)

        if len(approx) == 4:
            target = "Target Aquisition Successful"

        elif len(approx) == 3 or len(approx) == 5:
            target = "Partial Target Acquisition"

        return target

    def CLAHE(self, cv_image):
        '''
        CLAHE (Contrast Limited Adaptive Histogram Equalization)
        This increases the contrast between color channels and allows us to
        better differentiate colors under certain lighting conditions.
        '''
        clahe = cv2.createCLAHE(clipLimit=1., tileGridSize=(4, 4))

        # convert from BGR to LAB color space
        lab = cv2.cvtColor(cv_image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)  # split on 3 different channels

        l2 = clahe.apply(l)  # apply CLAHE to the L-channel

        lab = cv2.merge((l2, a, b))  # merge channels
        cv_image = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        return cv_image

    def mask_image(self, cv_image, lower, upper):
        mask = cv2.inRange(cv_image, lower, upper)
        # Remove anything not within the bounds of our mask
        output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # Resize to emphasize shapes
        resized = cv2.resize(output, (300, 225))
        ratio = output.shape[0] / float(resized.shape[0])
        gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

        # Blur image so our contours can better find the full shape.
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        return blurred, ratio

    def acquire_targets(self, cv_image):
        # Take in the data and get its dimensions.
        height, width, channels = cv_image.shape

        # Run CLAHE.
        cv_image = self.CLAHE(cv_image)

        # Now we generate a color mask to isolate only red in the image. This
        # is achieved through the thresholds which can be changed in the above
        # constants.

        # create NumPy arrays from the boundaries
        lower = np.array(self.lower, dtype="uint8")
        upper = np.array(self.upper, dtype="uint8")

        # Generate a mask based on the constants.
        blurred, ratio = self.mask_image(cv_image, lower, upper)

        # Compute contours
        cnts = cv2.findContours(blurred.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)

        cnts = cnts[1]
        '''
        We use OpenCV to compute our contours and then begin processing them
        to ensure we are identifying a proper target.
        '''

        shape = ''
        peri_max = 0
        max_x = 0
        max_y = 0
        m_shape = ''

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
            if shape == "Target Aquisition Successful" or \
               shape == "Partial Target Acquisition" or \
               shape == "unidentified":
                if self.debug:
                    try:
                        cv2.drawContours(cv_image, [c], -1, (0, 255, 0), 2)
                        cv2.putText(cv_image, shape, (cX, cY),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (255, 255, 255), 2)
                        self.image_pub.publish(cv_image)
                    except CvBridgeError as e:
                        print(e)

                peri = cv2.arcLength(c, True)
                if peri > peri_max:
                    peri_max = peri
                    max_x = cX
                    max_y = cY
                    m_shape = shape
        '''
        This is Kevin's Code, adapted for this project. We are trying to find
        the 3D coordinates of the torpedo board/target to give us a better idea
        of where we are trying to go and perform more accurate movements
        to align with the target. The first thing we need to do is convert from
        camera coordinates in pixels to 3D coordinates.
        Every time we succesfully get a target aquisition we add it to the
        counter. Once we observe it enough times
        we can be confident we are looking at the correct target. We then
        perform an least squares intersection from multiple angles
        to derive the approximate 3D coordinates.
        '''

        if m_shape == "Target Aquisition Successful" or \
                m_shape == "Partial Target Acquisition" or \
                m_shape == "unidentified":
            try:
                self.tf_listener.waitForTransform('/map',
                                                  self.camera_model.tfFrame(),
                                                  self.last_image_time,
                                                  rospy.Duration(0.2))
            except tf.Exception as e:
                rospy.logwarn(
                    "Could not transform camera to map: {}".format(e))
                return False

            (t, rot_q) = self.tf_listener.lookupTransform(
                '/map', self.camera_model.tfFrame(), self.last_image_time)
            R = mil_ros_tools.geometry_helpers.quaternion_matrix(rot_q)
            center = np.array([max_x, max_y])
            self.add_observation(center, (np.array(t), R),
                                 self.last_image_time)

            observations, pose_pairs = self.get_observations_and_pose_pairs()
            if len(observations) > self.min_observations:
                self.est = self.multi_obs.lst_sqr_intersection(
                    observations, pose_pairs)
                self.status = 'Pose found'

            else:
                self.status = '{} observations'.format(len(observations))


def main(args):
    rospy.init_node('torp_vision', anonymous=False)
    torp_vision()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
