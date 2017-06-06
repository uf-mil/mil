#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from image_geometry import PinholeCameraModel
import mil_ros_tools
import tf
from collections import deque
from sub8_vision_tools import rviz, MultiObservation
from sub8_msgs.srv import VisionRequest2DResponse, VisionRequest2D, VisionRequest, VisionRequestResponse
from std_msgs.msg import Header
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Pose2D, PoseStamped, Pose, Point
from mil_ros_tools import Image_Subscriber, Image_Publisher


class Buoy(object):
    '''
    Represents one colored buoy for use in BuoyFinder. Contains
    color values to segment an image for its color and an internal
    buffer of observations to use in position estimation.
    '''
    def __init__(self, color, debug_cv=False):
        self._observations = deque()
        self._pose_pairs = deque()
        self._times = deque()
        self.last_t = None
        self.debug_cv = debug_cv
        self.status = ''
        self.est = None
        self.color = color

        self.timeout = rospy.Duration(rospy.get_param('~timeout_seconds'))
        self.min_trans = rospy.get_param('~min_trans')

        # Only for visualization
        if color == 'red':
            self.draw_colors = (1.0, 0.0, 0.0, 1.0)
            self.cv_colors = (0, 0, 255, 0)
            self.visual_id = 0
        elif color == 'green':
            self.draw_colors = (0.0, 1.0, 0.0, 1.0)
            self.cv_colors = (0, 255, 0, 0)
            self.visual_id = 1
        elif color == 'yellow':
            self.draw_colors = (1.0, 1.0, 0.0, 1.0)
            self.cv_colors = (0, 255, 255, 0)
            self.visual_id = 2
        else:
            rospy.logerr('Unknown buoy color {}'.format(color))
            self.draw_colors = (0.0, 0.0, 0.0, 1.0)
            self.visual_id = 3
        if self.debug_cv:
            cv2.namedWindow(self.color)

    def load_segmentation(self):
        '''
        Load threshold values in BGR, HSV, or LAB colorspace for segmenting an image.
        '''
        for color_space in ['hsv', 'bgr', 'lab']:
            self.color_space = color_space
            low = '/color/buoy/{}/{}_low'.format(self.color, color_space)
            high = '/color/buoy/{}/{}_high'.format(self.color, color_space)
            if not rospy.has_param(low):
                continue
            self.thresholds = [np.array(rospy.get_param(low)),
                               np.array(rospy.get_param(high))]
            rospy.loginfo("BUOY - Thresholds for {} buoy loaded using {} colorspace".format(
                          self.color, self.color_space))

        if self.debug_cv:
            # If debug_cv is enabled, create trackbars to adjust thresholds for this buoy
            def change_threshold(i, i2, val):
                self.thresholds[i][i2] = float(val)
                rospy.loginfo("SETTING {} thresholds[{}][{}]={}".format(self.color, i, i2, self.thresholds[i][i2]))
            for i in range(len(self.thresholds[0])):
                cv2.createTrackbar('low {}'.format(self.color_space[i]), self.color, int(self.thresholds[0][i]), 255,
                                   lambda x, _ind=i: change_threshold(0, _ind, x))
            for i in range(len(self.thresholds[1])):
                cv2.createTrackbar('high {}'.format(self.color_space[i]), self.color, int(self.thresholds[1][i]), 255,
                                   lambda x, _ind=i: change_threshold(1, _ind, x))

    def get_mask(self, img):
        '''
        Use loaded threshold values to create a mask of img
        '''
        if self.color_space == 'hsv':
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, *self.thresholds)
        elif self.color_space == 'lab':
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            mask = cv2.inRange(lab, *self.thresholds)
        else:
            mask = cv2.inRange(img, *self.thresholds)
        return mask

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

    def add_observation(self, obs, pose_pair, time):
        self.clear_old_observations()
        if self.size() == 0 or np.linalg.norm(self._pose_pairs[-1][0] - pose_pair[0]) > self.min_trans:
            self._observations.append(obs)
            self._pose_pairs.append(pose_pair)
            self._times.append(time)

    def get_observations_and_pose_pairs(self):
        self.clear_old_observations()
        return (self._observations, self._pose_pairs)

    def size(self):
        return len(self._observations)

    def clear(self):
        self._times.clear()
        self._observations.clear()
        self._pose_pairs.clear()


class BuoyFinder(object):
    '''
    Node to find red, green, and yellow buoys in a single camera frame.

    Combines several observations and uses a least-squares approach to get a 3D
    position estimate of a buoy when requested.

    Intended to be modular so other approaches can be tried. Adding more sophistication
    to segmentation would increase reliability.
    '''
    def __init__(self):
        self.tf_listener = tf.TransformListener()

        self.enabled = False
        self.last_image = None
        self.last_image_time = None
        self.camera_model = None

        # Various constants for tuning, debugging. See buoy_finder.yaml for more info
        self.min_observations = rospy.get_param('~min_observations')
        self.max_observations = rospy.get_param('~max_observations')
        self.debug_ros = rospy.get_param('~debug/ros', True)
        self.debug_cv = rospy.get_param('~debug/cv', False)
        self.min_contour_area = rospy.get_param('~min_contour_area')
        self.max_velocity = rospy.get_param('~max_velocity')
        camera = rospy.get_param('~camera_topic', '/camera/front/right/image_rect_color')

        self.buoys = {}
        for color in ['red', 'yellow', 'green']:
            self.buoys[color] = Buoy(color, debug_cv=self.debug_cv)
            self.buoys[color].load_segmentation()
        if self.debug_cv:
            cv2.waitKey(1)
            self.debug_images = {}

        self.image_sub = Image_Subscriber(camera, self.image_cb)
        if self.debug_ros:
            self.rviz = rviz.RvizVisualizer(topic='~markers')
            self.mask_pub = Image_Publisher('~mask_image')
            rospy.Timer(rospy.Duration(1), self.print_status)

        self.camera_info = self.image_sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        self.frame_id = self.camera_model.tfFrame()
        self.multi_obs = MultiObservation(self.camera_model)

        rospy.Service('~enable', SetBool, self.toggle_search)
        rospy.Service('~2D', VisionRequest2D, self.request_buoy)
        rospy.Service('~pose', VisionRequest, self.request_buoy3d)

        rospy.loginfo("BUOY FINDER: initialized successfully")

    def toggle_search(self, srv):
        '''
        Callback for standard ~enable service. If true, start
        looking at frames for buoys.
        '''
        if srv.data:
            rospy.loginfo("BUOY FINDER: enabled")
            self.enabled = True
        else:
            rospy.loginfo("BUOY FINDER: disabled")
            self.enabled = False

        return SetBoolResponse(success=True)

    def request_buoy(self, srv):
        '''
        Callback for 2D vision request. Returns centroid
        of buoy found with color specified in target_name
        if found.
        '''
        if not self.enabled or srv.target_name not in self.buoys:
            return VisionRequest2DResponse(found=False)
        response = self.find_single_buoy(srv.target_name)
        if response is False or response is None:
            return VisionRequest2DResponse(found=False)
        center, radius = response
        return VisionRequest2DResponse(
            header=Header(stamp=self.last_image_time, frame_id=self.frame_id),
            pose=Pose2D(
                x=center[0],
                y=center[1],
            ),
            max_x=self.last_image.shape[0],
            max_y=self.last_image.shape[1],
            camera_info=self.image_sub.camera_info,
            found=True
        )

    def request_buoy3d(self, srv):
        '''
        Callback for 3D vision request. Uses recent observations of buoy
        specified in target_name to attempt a least-squares position estimate.
        As buoys are spheres, orientation is meaningless.
        '''
        if srv.target_name not in self.buoys or not self.enabled:
            return VisionRequestResponse(found=False)
        buoy = self.buoys[srv.target_name]
        if buoy.est is None:
            return VisionRequestResponse(found=False)
        return VisionRequestResponse(
            pose=PoseStamped(
                header=Header(stamp=self.last_image_time, frame_id='/map'),
                pose=Pose(
                    position=Point(*buoy.est)
                )
            ),
            found=True
        )

    def image_cb(self, image):
        '''
        Run each time an image comes in from ROS. If enabled,
        attempt to find each color buoy.
        '''
        if not self.enabled:
            return

        # Crop out some of the top and bottom to exclude the floor and surface reflections
        height = image.shape[0]
        roi_y = int(0.2 * height)
        roi_height = height - int(0.2 * height)
        self.roi = (0, roi_y, roi_height, image.shape[1])
        self.last_image = image[self.roi[1]:self.roi[2], self.roi[0]:self.roi[3]]

        if self.debug_ros:
            # Create a blacked out debug image for putting masks in
            self.mask_image = np.zeros(self.last_image.shape, dtype=image.dtype)
        if self.last_image_time is not None and self.image_sub.last_image_time < self.last_image_time:
            # Clear tf buffer if time went backwards (nice for playing bags in loop)
            self.tf_listener.clear()
        self.last_image_time = self.image_sub.last_image_time
        self.find_buoys()
        if self.debug_ros:
            self.mask_pub.publish(self.mask_image)

    def print_status(self, _):
        '''
        Called at 1 second intervals to display the status (not found, n observations, FOUND)
        for each buoy.
        '''
        if self.enabled:
            rospy.loginfo("STATUS: RED='%s', GREEN='%s', YELLOW='%s'",
                          self.buoys['red'].status,
                          self.buoys['green'].status,
                          self.buoys['yellow'].status)

    def find_buoys(self):
        '''
        Run find_single_buoy for each color of buoy
        '''
        for buoy_name in self.buoys:
            self.find_single_buoy(buoy_name)

    def get_best_contour(self, contours):
        '''
        Attempts to find a good buoy contour among those found within the
        thresholded mask. If a good one is found, it return (contour, centroid, area),
        otherwise returns None. Right now the best contour is just the largest.

        TODO: Use smarter contour filtering methods, like checking this it is circle like
        '''
        if len(contours) > 0:
            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            if area < self.min_contour_area:
                return None
            M = cv2.moments(cnt)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            tpl_center = (int(cx), int(cy))
            return cnt, tpl_center, area
        else:
            return None

    def find_single_buoy(self, buoy_type):
        '''
        Attempt to find one color buoy in the image.
        1) Create mask for buoy's color in colorspace specified in paramaters
        2) Select the largest contour in this mask
        3) Approximate a circle around this contour
        4) Store the center of this circle and the current tf between /map and camera
           as an observation
        5) If observations for this buoy is now >= min_observations, approximate buoy
           position using the least squares tool imported
        '''
        assert buoy_type in self.buoys.keys(), "Buoys_2d does not know buoy color: {}".format(buoy_type)
        buoy = self.buoys[buoy_type]
        mask = buoy.get_mask(self.last_image)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=2)
        mask = cv2.dilate(mask, kernel, iterations=2)

        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE,
                                          offset=(self.roi[0], self.roi[1]))
        ret = self.get_best_contour(contours)
        if ret is None:
            buoy.clear_old_observations()
            buoy.status = 'not seen w/ {} obs'.format(buoy.size())
            return
        contour, tuple_center, area = ret
        true_center, rad = cv2.minEnclosingCircle(contour)

        if self.debug_ros:
            cv2.add(self.mask_image.copy(), buoy.cv_colors, mask=mask, dst=self.mask_image)
            cv2.circle(self.mask_image, (int(true_center[0] - self.roi[0]), int(true_center[1]) - self.roi[1]),
                       int(rad), buoy.cv_colors, 2)
        if self.debug_cv:
            self.debug_images[buoy_type] = mask.copy()

        try:
            self.tf_listener.waitForTransform('/map', self.frame_id, self.last_image_time, rospy.Duration(0.2))
        except tf.Exception as e:
            rospy.logwarn("Could not transform camera to map: {}".format(e))
            return False

        if not self.sanity_check(tuple_center, self.last_image_time):
            buoy.status = 'failed sanity check'
            return False

        (t, rot_q) = self.tf_listener.lookupTransform('/map', self.frame_id, self.last_image_time)
        R = mil_ros_tools.geometry_helpers.quaternion_matrix(rot_q)

        buoy.add_observation(true_center, (np.array(t), R), self.last_image_time)

        observations, pose_pairs = buoy.get_observations_and_pose_pairs()
        if len(observations) > self.min_observations:
            buoy.est = self.multi_obs.lst_sqr_intersection(observations, pose_pairs)
            buoy.status = 'Pose found'
            if self.debug_ros:
                self.rviz.draw_sphere(buoy.est, color=buoy.draw_colors,
                                      scaling=(0.2286, 0.2286, 0.2286),
                                      frame='/map', _id=buoy.visual_id)
        else:
            buoy.status = '{} observations'.format(len(observations))

        return tuple_center, rad

    def sanity_check(self, coordinate, timestamp):
        '''
        Check if the observation is unreasonable. More can go here if we want.
        '''
        sane = True
        velocity = np.linalg.norm(self.tf_listener.lookupTwist('/map', self.frame_id, timestamp, rospy.Duration(.5)))
        if velocity > self.max_velocity:
            sane = False
        return sane

if __name__ == '__main__':
    rospy.init_node('buoy_finder')
    b = BuoyFinder()

    if b.debug_cv:
        # Keep opencv gui alive as ros spins
        cv2.waitKey(1)
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            for k in b.debug_images:
                cv2.imshow(k, b.debug_images[k])
            cv2.waitKey(1)
            r.sleep()
    else:
        rospy.spin()
