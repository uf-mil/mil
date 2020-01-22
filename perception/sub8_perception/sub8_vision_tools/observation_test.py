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
from geometry_msgs.msg import PoseStamped, Pose, Point
from sub8_msgs.srv import VisionRequest, VisionRequestResponse
from mil_ros_tools import Image_Subscriber, Image_Publisher
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker

class TestObs:

    def __init__(self, cam='/camera/front/left/image_rect_color', cam_optical='/front_left_cam_optical'):
        self.cam = cam
        self.cam_optical = cam_optical
        self.tf_listener = tf.TransformListener()
        print(cam)
        self.roi_pub = rospy.Publisher('roi_pub', RegionOfInterest, queue_size=1)
        self.last_image_time = None
        self.image_sub = Image_Subscriber(cam, self.image_cb)
        self.camera_info = self.image_sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        self.camera_matrix = np.asarray(self.camera_info.P).reshape(3, 4)

        self.markerPub0 = rospy.Publisher('pointMarker0', Marker, queue_size=1)
        self.markerPub1 = rospy.Publisher('pointMarker1', Marker, queue_size=1)
        self.markerPub2 = rospy.Publisher('pointMarker2', Marker, queue_size=1)
        self.markerPub3 = rospy.Publisher('pointMarker3', Marker, queue_size=1)

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

        robotMarker.color.r = 0.0
        robotMarker.color.g = 1.0
        robotMarker.color.b = 0.0
        robotMarker.color.a = 1.0

        robotMarker.lifetime = rospy.Duration(0)
        return robotMarker

    def image_cb(self, msg):
        # rospy.sleep(.25)
        if self.last_image_time is not None and \
            self.image_sub.last_image_time < self.last_image_time:
            print('clear')
            # Clear tf buffer if time went backwards (nice for playing bags in
            # loop)
            self.tf_listener.clear()
        try:
            print("Time: ", self.image_sub.last_image_time)
            self.tf_listener.waitForTransform(self.cam_optical,
                                            '/map',
                                            self.image_sub.last_image_time,
                                            rospy.Duration(.2))
        except tf.Exception as e:
            rospy.logwarn(
                "Could not transform camera to map: {}".format(e))
        self.last_image_time = self.image_sub.last_image_time
        (t, rot_q) = self.tf_listener.lookupTransform(self.cam_optical, '/map', self.image_sub.last_image_time)
        print(t, rot_q)
        R = mil_ros_tools.geometry_helpers.quaternion_matrix(rot_q)
        map_0 = np.array([.25,.25,-1])
        self.markerPub0.publish(self.marker_msg(map_0, 'map_0'))
        map_0 = R.dot(map_0) + t
        map_0 = (self.camera_matrix.dot(np.append(map_0, 1).T)).T
        x0 = map_0[0] / map_0[2]
        y0 = map_0[1] / map_0[2]

        map_1 = np.array([.25,1,-1])
        self.markerPub1.publish(self.marker_msg(map_1, 'map_1'))
        map_1 = R.dot(map_1) + t
        map_1 = (self.camera_matrix.dot(np.append(map_1, 1).T)).T
        x1 = map_1[0] / map_1[2]
        y1 = map_1[1] / map_1[2]

        map_2 = np.array([1,.25,-1])
        self.markerPub2.publish(self.marker_msg(map_2, 'map_2'))
        map_2 = R.dot(map_2) + t
        map_2 = (self.camera_matrix.dot(np.append(map_2, 1).T)).T
        x2 = map_2[0] / map_2[2]
        y2 = map_2[1] / map_2[2]
        # print(np.append(map_2, 1))

        map_3 = np.array([1,1,-1])
        self.markerPub3.publish(self.marker_msg(map_3, 'map_3'))
        map_3 = R.dot(map_3) + t
        map_3 = (self.camera_matrix.dot(np.append(map_3, 1).T)).T
        x3 = map_3[0] / map_3[2]
        y3 = map_3[1] / map_3[2]

        # print('map_0 = ', map_0)
        # print('map_1 = ', map_1)
        # print('map_2 = ', map_2)
        # print('map_3 = ', map_3)

        print('x1 = ', x0)
        print('x2 = ', x1)
        print('x3 = ', x2)
        print('x4 = ', x3)

        print('y1 = ', y0)
        print('y2 = ', y1)
        print('y3 = ', y2)
        print('y4 = ', y3)

        leastx = min(x0, x1, x2, x3)
        leasty = min(y0, y1, y2, y3)

        maxx = max(x0, x1, x2, x3)
        maxy = max(y0, y1, y2, y3)

        roi = RegionOfInterest(x_offset=leastx, y_offset=leasty, height=int(-(leasty - maxy)), width=int(-(leastx - maxx)))
        self.roi_pub.publish(roi)

        
def main(args):
    if len(args) < 2:
        print('Need camera... observation_test.py [left, right, down]')
        return
    rospy.init_node('observation_test', anonymous=True)
    if args[1] == 'left':
        cam='/camera/front/left/image_rect_color'
        cam_optical='/front_left_cam_optical'   
    if args[1] == 'right':
        cam='/camera/front/left/image_rect_color'
        cam_optical='/front_right_cam_optical'   
    if args[1] == 'down':
        cam='/camera/down/image_rect_color'
        cam_optical='/down_left_cam_optical'   

    TestObs(cam, cam_optical)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
