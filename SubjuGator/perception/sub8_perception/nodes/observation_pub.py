#!/usr/bin/env python
from __future__ import print_function
import sys
import tf
import mil_ros_tools
import cv2
import numpy as np
import rospy
from std_msgs.msg import Header
from collections import deque
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridgeError
from sub8_vision_tools import MultiObservation
from image_geometry import PinholeCameraModel
from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sub8_msgs.srv import VisionRequest, VisionRequestResponse
from mil_ros_tools import Image_Subscriber, Image_Publisher
from cv_bridge import CvBridge
from mil_misc_tools import FprintFactory
from visualization_msgs.msg import Marker


'''
Perception component of the Torpedo Board Challenge. Utilizes code from
the pyimagesearch blog post on color thresholding and shape detection
as well as code from the buoy_finder mission of previous years.
'''
MISSION='PERCEPTION'


class MultiObs:

    def __init__(self):

        # Pull constants from config file
        self.min_trans = rospy.get_param('~min_trans', .25)
        self.max_velocity = rospy.get_param('~max_velocity', 1)
        self.min_observations = rospy.get_param('~min_observations', 30)
        self.camera = rospy.get_param('~camera_topic',
                                      '/camera/front/left/image_rect_color')
        # Instantiate remaining variables and objects
        self._observations = deque()
        self._pose_pairs = deque()
        self._times = deque()
        self._observations1 = deque()
        self._pose_pairs1 = deque()
        self._times1 = deque()
        self._observations2 = deque()
        self._pose_pairs2 = deque()
        self._times2 = deque()
        self.last_image_time = None
        self.last_image = None
        self.tf_listener = tf.TransformListener()
        self.status = ''
        self.est = None
        self.est1 = None
        self.est2 = None
        self.plane = None
        self.visual_id = 0
        self.enabled = False
        self.bridge = CvBridge() 
        self.print_info = FprintFactory(title=MISSION).fprint
        # Image Subscriber and Camera Information
        self.point_sub = rospy.Subscriber(
            '/roi_pub', RegionOfInterest, self.acquire_targets)
        self.image_sub = Image_Subscriber(self.camera, self.image_cb)

        self.camera_info = self.image_sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        self.frame_id = 'front_left_cam_optical'
        self.markerPub0 = rospy.Publisher('estMarker0', Marker, queue_size=1)
        self.markerPub1 = rospy.Publisher('estMarker1', Marker, queue_size=1)
        self.markerPub2 = rospy.Publisher('estMarker2', Marker, queue_size=1)
        # Ros Services so mission can be toggled and info requested
        rospy.Service('~enable', SetBool, self.toggle_search)
        self.multi_obs = MultiObservation(self.camera_model)
        rospy.Service('~pose', VisionRequest, self.request_board3d)

        # Debug
        self.debug = rospy.get_param('~debug', True)

    def image_cb(self, image):
        '''
        Run each time an image comes in from ROS. If enabled,
        attempt to find the torpedo board.
        '''
        return None

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
          print("REEEE")  
          return VisionRequestResponse(found=False)
        #buoy = self.buoys[srv.target_name]
        if self.est is None:
            self.print_info("NO ESTIMATE!")
            return VisionRequestResponse(found=False)
        # NOTE: returns normal vec encoded into a quaternion message (so not actually a quaternion)
        return VisionRequestResponse(
            pose=PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='/map'),
                pose=Pose(position=Point(*((self.est + self.est1 + self.est2) / 3)),
                          orientation=Quaternion(x=self.plane[0], y=self.plane[1], z=self.plane[2]))),
                found=True)

    def clear_old_observations(self):
        # Observations older than three seconds are discarded.
        # print(self._observations)
        # print(self._observations1)
        # print(self._observations2)
        return

    def size(self):
        return len(self._observations)

    def add_observation(self, obs, pose_pair, time):
        if len(self._pose_pairs) == 0 or np.linalg.norm(
                self._pose_pairs[-1][0] - pose_pair[0]) > self.min_trans:
            self._observations.append(obs)
            self._pose_pairs.append(pose_pair)
            self._times.append(time)

    def add_observation1(self, obs, pose_pair, time):
        if len(self._pose_pairs1) == 0 or np.linalg.norm(
                self._pose_pairs1[-1][0] - pose_pair[0]) > self.min_trans:
            self._observations1.append(obs)
            self._pose_pairs1.append(pose_pair)
            self._times1.append(time)
        return (self._observations1, self._pose_pairs1)

    def add_observation2(self, obs, pose_pair, time):
        self.clear_old_observations()
        if len(self._pose_pairs2) == 0 or np.linalg.norm(
                self._pose_pairs2[-1][0] - pose_pair[0]) > self.min_trans:
            self._observations2.append(obs)
            self._pose_pairs2.append(pose_pair)
            self._times2.append(time)
        return (self._observations2, self._pose_pairs2)

    def get_observations_and_pose_pairs(self):
        self.clear_old_observations()
        return (self._observations, self._pose_pairs)

    def generate_plane(self):
        ab = self.est - self.est1
        ac = self.est - self.est2
        # print("AB: ", ab)
        # print("AC: ", ac)
        x = np.cross(ab,ac)
        return x / np.linalg.norm(x)

    def marker_msg(self, point, point_name):
        robotMarker = Marker()
        robotMarker.header.frame_id = "/map"
        robotMarker.header.stamp    = rospy.get_rostime()
        robotMarker.ns = point_name
        robotMarker.id = 0
        robotMarker.type = 2 # sphere
        robotMarker.action = 0
        robotMarker.pose.position = Point(point[0], point[1], point[2])
        robotMarker.pose.orientation.x = 0
        robotMarker.pose.orientation.y = 0
        robotMarker.pose.orientation.z = 0
        robotMarker.pose.orientation.w = 1.0
        robotMarker.scale.x = .5
        robotMarker.scale.y = .5
        robotMarker.scale.z = .5

        robotMarker.color.r = 1.0
        robotMarker.color.g = 1.0
        robotMarker.color.b = 0.0
        robotMarker.color.a = 1.0

        robotMarker.lifetime = rospy.Duration(0)
        return robotMarker

    def acquire_targets(self, roi):
        if not self.enabled:
          return
        # NOTE: point.z contains the timestamp of the image when it was processed in the neural net.
        x0 = roi.x_offset
        y0 = roi.y_offset
        height = roi.height
        width = roi.width
        
        point0 = np.array([x0, y0])
        point1 = np.array([x0+width, y0])
        point2 = np.array([x0, y0+height])
        point3 = np.array([x0+width, y0+height])
        # print("p1: ", point1)
        # print("p2: ", point2)
        try:
            self.tf_listener.waitForTransform('/map',
                                                '/front_left_cam_optical',
                                                self.image_sub.last_image_time - rospy.Time(.2),
                                                rospy.Duration(0.2))
        except tf.Exception as e:
            rospy.logwarn(
                "Could not transform camera to map: {}".format(e))
            return False

        (t, rot_q) = self.tf_listener.lookupTransform(
            '/map', '/front_left_cam_optical', self.image_sub.last_image_time - rospy.Time(.2))
        R = mil_ros_tools.geometry_helpers.quaternion_matrix(rot_q)

        self.add_observation(point0, (np.array(t), R),
                                self.image_sub.last_image_time)
        observations1, pose_pairs1 = self.add_observation1(point1, (np.array(t), R),
                                self.image_sub.last_image_time)
        observations2, pose_pairs2 = self.add_observation2(point2, (np.array(t), R),
                                self.image_sub.last_image_time)

        observations, pose_pairs = self.get_observations_and_pose_pairs()
        if len(observations) > self.min_observations:
            self.est = self.multi_obs.lst_sqr_intersection(
                observations, pose_pairs)
            self.est1 = self.multi_obs.lst_sqr_intersection(
                observations1, pose_pairs1)
            self.est2 = self.multi_obs.lst_sqr_intersection(
                observations2, pose_pairs2)
            # print('est1: ', self.est1)
            # print('est2: ', self.est2)
            self.status = 'Pose found'
            self.print_info("Pose Found!")
            self.plane = self.generate_plane()
            # print("Plane: ", self.plane)
            self.markerPub0.publish(self.marker_msg(self.est, 'est0'))
            self.markerPub1.publish(self.marker_msg(self.est1, 'est1'))
            self.markerPub2.publish(self.marker_msg(self.est2, 'est2'))
            print((self.est + self.est1 + self.est2) / 3)

        else:   
            self.status = '{} observations'.format(len(observations))


def main(args):
    rospy.init_node('xyz_points', anonymous=False)
    MultiObs()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
