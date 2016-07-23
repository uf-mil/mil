#!/usr/bin/env python
import cv2
import numpy as np
import sys
import rospy
import rospkg
import sub8_ros_tools
import tf
import os
from std_msgs.msg import Header
from std_srvs.srv import SetBool, SetBoolResponse
from sub8_msgs.srv import VisionRequestResponse, VisionRequest
from geometry_msgs.msg import PoseStamped
from sub8_vision_tools import MarkerOccGrid, rviz, machine_learning
from sub8_ros_tools import numpy_quat_pair_to_pose
from image_geometry import PinholeCameraModel

SEARCH_DEPTH = .65  # m

# Boost files (maybe move to launch?) - I put these here so they're easy to change
boost_to_the_moon = True
MARKER = 'red_gentle_3tree_5depth.dic'

rospack = rospkg.RosPack()

class MarkerFinder():
    def __init__(self):
        self.tf_listener = tf.TransformListener()

        self.search = False
        self.last_image = None
        self.last_image_timestamp = None
        self.last_draw_image = None

        # This may need to be changed if you want to use a different image feed.
        self.image_sub = sub8_ros_tools.Image_Subscriber('/down/left/image_raw', self.image_cb)
        self.image_pub = sub8_ros_tools.Image_Publisher("vision/channel_marker/target_info")

        self.toggle = rospy.Service('vision/channel_marker/search', SetBool, self.toggle_search)

        self.cam = PinholeCameraModel()
        self.cam.fromCameraInfo(self.image_sub.wait_for_camera_info())
        self.rviz = rviz.RvizVisualizer()

        # self.occ_grid = MarkerOccGrid(self.image_sub, grid_res=.05, grid_width=500, grid_height=500,
        #                               grid_starting_pose=Pose2D(x=250, y=250, theta=0))
        if boost_to_the_moon:
            self.boost = cv2.Boost()
            rospy.loginfo("MARKER - Loading boost...")
            self.boost.load(os.path.join(rospack.get_path('sub8_perception'), 'ml_classifiers/marker/' + MARKER))
            rospy.loginfo("MARKER - Classifier for marker loaded.")
        else:
            self.lower = np.array(rospy.get_param('channel_guide/hsv_low'))
            self.upper = np.array(rospy.get_param('channel_guide/hsv_high'))

        self.pose_service = rospy.Service("vision/channel_marker/pose", VisionRequest, self.request_marker)

        self.kernel = np.ones((15, 15), np.uint8)

        # Occasional status publisher
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_target_info)

        print "MARKER - Got no patience for sittin' around!"


    def toggle_search(self, srv):
        if srv.data:
            rospy.loginfo("MARKER - Looking for markers now.")
            self.search = True
        else:
            rospy.loginfo("MARKER - Done looking for markers.")
            self.search = False

        return SetBoolResponse(success=srv.data)

    def publish_target_info(self, *args):
        if not self.search or self.last_image is None:
            return

        markers = self.find_marker(np.copy(self.last_image))
        #self.occ_grid.update_grid(self.last_image_timestamp)
        #self.occ_grid.add_marker(markers, self.last_image_timestamp)

        if self.last_draw_image is not None: #and (markers is not None):
            self.image_pub.publish(np.uint8(self.last_draw_image))

    def image_cb(self, image):
        '''Hang on to last image and when it was taken.'''
        self.last_image = image
        self.last_image_timestamp = self.image_sub.last_image_time

    def calculate_threshold(self, img, agression=.5):
        histr = cv2.calcHist([img], [0], None, [179], [0, 179])
        threshold = np.uint8((179 - np.argmax(histr)) * agression)
        return threshold

    def get_2d_pose(self, mask):
        # estimate covariance matrix and get corresponding eigenvectors
        wh = np.where(mask)[::-1]
        cov = np.cov(wh)
        eig_vals, eig_vects = np.linalg.eig(cov)

        # use index of max eigenvalue to find max eigenvector
        i = np.argmax(eig_vals)
        max_eigv = eig_vects[:, i] * np.sqrt(eig_vals[i])

        # flip indices to find min eigenvector
        min_eigv = eig_vects[:, 1 - i] * np.sqrt(eig_vals[1 - i])

        # define center of pipe
        center = np.average(wh, axis=1)

        # define vertical vector (sub's current direction)
        vert_vect = np.array([0.0, -1.0])

        if max_eigv[1] > 0:
            max_eigv = -max_eigv
            min_eigv = -min_eigv

        num = np.cross(max_eigv, vert_vect)
        denom = np.linalg.norm(max_eigv) * np.linalg.norm(vert_vect)
        angle_rad = np.arcsin(num / denom)

        return center, angle_rad, [max_eigv, min_eigv]

    def find_marker(self, img):
        #img[:, -100:] = 0
        #img = cv2.GaussianBlur(img, (7, 7), 15)
        last_image_timestamp = self.last_image_timestamp

        if boost_to_the_moon:
            some_observations = machine_learning.boost.observe(img)
            prediction = [int(x) for x in [self.boost.predict(obs) for obs in some_observations]]
            mask = np.reshape(prediction, img[:, :, 2].shape).astype(np.uint8) * 255
        else:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower, self.upper)


        kernel = np.ones((5,5),np.uint8)
        mask = cv2.dilate(mask, kernel, iterations = 1)
        mask = cv2.erode(mask, kernel, iterations = 2)

        #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(np.copy(mask), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) < 1:
            rospy.logwarn("MARKER - No marker found.")
            return None

        # Find biggest area contour
        self.last_draw_image = np.dstack([mask] * 3)
        areas = [cv2.contourArea(c) for c in contours]
        max_index = np.argmax(areas)
        cnt = contours[max_index]

        # Draw a miniAreaRect around the contour and find the area of that.
        rect = cv2.minAreaRect(cnt)
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        mask = np.zeros(shape=mask.shape)
        cv2.drawContours(mask, [box], 0, 255, -1)
        rect_area = cv2.contourArea(box)

        center, angle_rad, [max_eigv, min_eigv] = self.get_2d_pose(mask)

        cv2.line(self.last_draw_image, tuple(np.int0(center)), tuple(np.int0(center + (2 * max_eigv))), (0, 255, 30), 2)
        cv2.line(self.last_draw_image, tuple(np.int0(center)), tuple(np.int0(center + (2 * min_eigv))), (0, 30, 255), 2)

        # Check if the box is too big or small.
        xy_position, height = self.get_tf(timestamp=last_image_timestamp)
        expected_area = self.calculate_marker_area(height)
        # print expected_area
        # print rect_area
        # if expected_area * .1 < rect_area < expected_area * 2:
        #     #cv2.drawContours(self.last_draw_image, [box], 0, (255, 255, 255), -1)
        #     self.rviz.draw_ray_3d(center, self.cam, np.array([1, .5, 0, 1]), frame='/downward',
        #         _id=5, length=height, timestamp=last_image_timestamp)
        # else:
        #     angle_rad = 0
        #     max_eigv = np.array([0, -20])
        #     min_eigv = np.array([-20, 0])
        #     #cv2.drawContours(self.last_draw_image, [box], 0, (255, 0, 30), -1)
        #     rospy.logwarn("MARKER - Size out of bounds!")

        self.rviz.draw_ray_3d(center, self.cam, np.array([1, .5, 0, 1]), frame='/downward',
            _id=5, length=height, timestamp=last_image_timestamp)

        # Convert to a 3d pose to move the sub to.
        abs_position = self.transform_px_to_m(center, last_image_timestamp)
        q_rot = tf.transformations.quaternion_from_euler(0, 0, angle_rad)

        return numpy_quat_pair_to_pose(abs_position, q_rot)

    def request_marker(self, data):
        if self.last_image is None:
            return False  # Fail if we have no images cached

        timestamp = self.last_image_timestamp
        goal_pose = self.find_marker(self.last_image)
        found = (goal_pose is not None)

        if not found:
            resp = VisionRequestResponse(
                found=found
            )
        else:
            resp = VisionRequestResponse(
                pose=PoseStamped(
                    header=Header(
                        stamp=timestamp,
                        frame_id='/down_camera'),
                    pose=goal_pose
                ),
                found=found
            )
        return resp

    def get_tf(self, timestamp=None, get_rotation=False):
        '''
        x_y position, height in meters and quat rotation of the sub if requested
        '''
        if timestamp is None:
            timestamp = rospy.Time()

        self.tf_listener.waitForTransform("/map", "/downward", timestamp, rospy.Duration(5.0))
        trans, rot = self.tf_listener.lookupTransform("/map", "/downward", timestamp)
        x_y_position = trans[:2]
        self.tf_listener.waitForTransform("/ground", "/downward", timestamp, rospy.Duration(5.0))
        trans, _ = self.tf_listener.lookupTransform("/ground", "/downward", timestamp)

        height = np.nan_to_num(trans[2])
        x_y_position = np.nan_to_num(x_y_position)

        if get_rotation:
            return x_y_position, rot, height

        return x_y_position, height

    def transform_px_to_m(self, m_position, timestamp):
        '''
        Finds the absolute position of the marker in meters.
        '''
        xy_position, q, height = self.get_tf(timestamp, get_rotation=True)

        dir_vector = unit_vector(np.array([self.cam.cx(), self.cam.cy()]) - m_position)
        cam_rotation = tf.transformations.euler_from_quaternion(q)[2] + np.pi / 2
        print "MARKER - dir_vector:", dir_vector
        print "MARKER - cam_rotation:", cam_rotation
        dir_vector = np.dot(dir_vector, make_2D_rotation(cam_rotation))

        # Calculate distance from middle of frame to marker in meters.
        magnitude = self.calculate_visual_radius(height, second_point=m_position)
        abs_position = np.append(xy_position + dir_vector[::-1] * magnitude, -SEARCH_DEPTH)

        return abs_position

    def calculate_visual_radius(self, height, second_point=None):
        '''
        Draws rays to find the radius of the FOV of the camera in meters.
        It also can work to find the distance between two planar points some distance from the camera.
        '''

        mid_ray = np.array([0, 0, 1])

        if second_point is None:
            if self.cam.cy() < self.cam.cx():
                second_point = np.array([self.cam.cx(), 0])
            else:
                second_point = np.array([0, self.cam.cy()])

        edge_ray = unit_vector(self.cam.projectPixelTo3dRay(second_point))

        # Calculate angle between vectors and use that to find r
        theta = np.arccos(np.dot(mid_ray, edge_ray))
        return np.tan(theta) * height

    def calculate_marker_area(self, height):
        '''
        What we really don't want is to find markers that are on the edge since the direction and center of
            the marker are off.
        '''
        MARKER_LENGTH = 1.22  # m
        MARKER_WIDTH = .1524  # m

        # Get m/px on the ground floor.
        m = self.calculate_visual_radius(height)
        if self.cam.cy() < self.cam.cx():
            px = self.cam.cy()
        else:
            px = self.cam.cx()

        m_px = m / px
        marker_area_m = MARKER_WIDTH * MARKER_LENGTH
        marker_area_px = marker_area_m / (m_px ** 2)

        return marker_area_px

def unit_vector(vect):
    return vect / np.linalg.norm(vect)


def make_2D_rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s],
                     [s, c]], dtype=np.float32)


def main(args):
    redniFrekraM = MarkerFinder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    rospy.init_node('channel_marker')
    main(sys.argv)
