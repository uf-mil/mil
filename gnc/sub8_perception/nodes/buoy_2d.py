#!/usr/bin/env python
import cv2
import numpy as np
import sys
import rospy
import image_geometry
import sub8_ros_tools
import tf
from sub8_vision_tools import machine_learning

from collections import deque
from sub8_vision_tools import threshold_tools, rviz, ProjectionParticleFilter, MultiObservation
from sub8_msgs.srv import VisionRequest2DResponse, VisionRequest2D, VisionRequest, VisionRequestResponse
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D, PoseStamped, Pose, Point


class BuoyFinder:
    _min_size = 50

    def __init__(self):
        self.transformer = tf.TransformListener()
        rospy.sleep(1.0)
        self.done_once = False

        self.last_image = None
        self.last_draw_image = None
        self.last_poop_image = None
        self.last_image_time = None
        self.camera_model = None
        self.last_t = None

        self.observations = deque()
        self.pose_pairs = deque()

        self.rviz = rviz.RvizVisualizer()

        self.pose2d_service = rospy.Service('vision/buoys/2D', VisionRequest2D, self.request_buoy)
        self.pose_service = rospy.Service('vision/buoys/pose', VisionRequest, self.request_buoy3d)

        self.image_sub = sub8_ros_tools.Image_Subscriber('/stereo/right/image_rect_color', self.image_cb)
        self.image_pub = sub8_ros_tools.Image_Publisher('/vision/buoy_2d/target_info')

        # Occasional status publisher
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_target_info)

        self.boost = cv2.Boost()
        rospy.loginfo("Loading boost")
        self.boost.load('/home/jacob/catkin_ws/src/Sub8/gnc/sub8_perception/sub8_vision_tools/machine_learning/boost_huge.cv2')
        rospy.loginfo("Boost loaded")

        self.buoys = {
            'green': '/color/buoy/green',
            'red': '/color/buoy/red',
            'yellow': '/color/buoy/yellow',
        }

        self.ppf = None
        self.multi_obs = None

        self.draw_colors = {
            'green': (0.0, 1.0, 0.0, 1.0),
            'red': (1.0, 0.0, 0.0, 1.0),
            'yellow': (1.0, 1.0, 0.0, 1.0),
        }

    def request_buoy(self, srv):
        print 'requesting', srv
        response = self.find_single_buoy(np.copy(self.last_image), srv.target_name)

        if response is False:
            print 'did not find'
            resp = VisionRequest2DResponse(
                header=sub8_ros_tools.make_header(frame='/stereo_front/right'),
                found=False
            )

        else:
            # Fill in
            center, radius = response
            resp = VisionRequest2DResponse(
                header=Header(stamp=self.last_image_time, frame_id='/stereo_front/right'),
                pose=Pose2D(
                    x=center[0],
                    y=center[1],
                ),
                max_x=self.last_image.shape[0],
                max_y=self.last_image.shape[1],
                camera_info=self.image_sub.camera_info,
                found=True
            )
        return resp

    def request_buoy3d(self, srv):
        print "Requesting 3d pose"

        if (len(self.observations) > 5) and self.multi_obs is not None:
            estimated_pose = self.multi_obs.multilaterate(self.observations, self.pose_pairs)
            self.rviz.draw_sphere(estimated_pose, color=(0.2, 0.8, 0.0, 1.0), scaling=(0.5, 0.5, 0.5), frame='/map')
            resp = VisionRequestResponse(
                pose=PoseStamped(
                    header=Header(stamp=self.last_image_time, frame_id='/map'),
                    pose=Pose(
                        position=Point(*estimated_pose)
                    )
                ),
                found=True
            )
        else:
            if len(self.observations) <= 5:
                rospy.logerr("Did not attempt search because we did not have enough observations")
            else:
                rospy.logerr("Did not attempt search because buoys_2d was not fully initialized")

            resp = VisionRequestResponse(
                pose=PoseStamped(
                    header=Header(stamp=self.last_image_time, frame_id='/map'),
                ),
                found=False
            )
        return resp

    def publish_target_info(self, *args):
        if self.last_image is None:
            return

        self.find_buoys(np.copy(self.last_image))
        if self.last_draw_image is not None:
            self.image_pub.publish(self.last_draw_image)

    def image_cb(self, image):
        '''Hang on to last image'''
        self.last_image = image
        self.last_image_time = self.image_sub.last_image_time
        if self.camera_model is None:
            if self.image_sub.camera_info is None:
                return

            self.camera_model = image_geometry.PinholeCameraModel()
            self.camera_model.fromCameraInfo(self.image_sub.camera_info)
            self.multi_obs = MultiObservation(self.camera_model)

    def ncc(self, image, mean_thresh, scale=15):
        '''Compute normalized cross correlation w.r.t a shadowed pillbox fcn

        The expected scale will vary, so we don't cache it
        '''
        kernel = np.ones((scale, scale)) * -1
        midpoint = (scale // 2, scale // 2)
        cv2.circle(kernel, midpoint, midpoint[0], 1, -1)

        mean, std_dev = cv2.meanStdDev(image)

        # Check if the scene is brighter than our a priori target
        if mean > mean_thresh:
            kernel = -kernel

        normalized_cross_correlation = cv2.filter2D((image - mean) / std_dev, -1, kernel)
        renormalized = normalized_cross_correlation
        return renormalized

    def get_biggest(self, contours):
        if len(contours) > 0:
            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            if area > self._min_size:
                M = cv2.moments(cnt)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                tpl_center = (int(cx), int(cy))
                return cnt, tpl_center, area
        else:
            return None

    def find_single_buoy(self, img, buoy_type):
        if buoy_type != 'yellow':
            return

        assert buoy_type in self.buoys[buoy_type], "Buoys_2d does not know buoy color: {}".format(buoy_type)
        max_area = 0
        best_ret = None

        some_observations = machine_learning.boost.observe(img)
        prediction2 = [int(x) for x in [self.boost.predict(obs) for obs in some_observations]]
        mask = np.reshape(prediction2, img[:, :, 2].shape).astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        ret = self.get_biggest(contours)
        if ret is None:
            return

        contour, tuple_center, area = ret
        if area > max_area:
            max_area = area
            best_ret = ret

        if best_ret is None:
            return False

        contour, tuple_center, area = best_ret
        true_center, rad = cv2.minEnclosingCircle(contour)

        if self.camera_model is not None and (buoy_type == 'yellow'):
            self.rviz.draw_ray_3d(tuple_center, self.camera_model, self.draw_colors[buoy_type])
            (t, rot_q) = self.transformer.lookupTransform('/map', '/stereo_front/right', self.last_image_time - rospy.Duration(0.14))
            trans = np.array(t)
            R = sub8_ros_tools.geometry_helpers.quaternion_matrix(rot_q)

            if (self.last_t is None) or (np.linalg.norm(trans - self.last_t) > 0.3):
                self.last_t = trans
                self.observations.append(true_center)
                self.pose_pairs.append((t, R))

            if len(self.observations) > 5:
                est = self.multi_obs.multilaterate(self.observations, self.pose_pairs)
                self.rviz.draw_sphere(est, color=(0.9, 0.1, 0.0, 1.0), scaling=(0.3, 0.3, 0.3), frame='/map')

            if len(self.observations) > 10:
                self.observations.popleft()
                self.pose_pairs.popleft()

        return tuple_center, rad

    def find_buoys(self, img):
        draw_image = np.copy(img)

        # This is only run if buoy_type is not None
        for buoy_name in self.buoys.keys():
            result = self.find_single_buoy(img, buoy_name)
            if not result:
                continue

            center, rad = result
            cv2.circle(draw_image, center, int(rad), (255, 255, 0), 2)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(draw_image, '{}'.format(buoy_name), center, font, 0.8, (20, 20, 240), 1)

        self.last_draw_image = np.copy(draw_image)


def main(args):
    bf = BuoyFinder()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('orange_pipe_vision')
    main(sys.argv)
