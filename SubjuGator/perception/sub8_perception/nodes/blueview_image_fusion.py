#!/usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np

import cv2

import tf
from mil_blueview_driver.msg import BlueViewPing
from mil_ros_tools import Image_Publisher, StereoImageSubscriber, project_points3d_to_pixels_depths
from mil_ros_tools import rotate_vect_by_quat

class ImageBlueviewFussion (StereoImageSubscriber):
    def __init__(self, left_image_topic, right_image_topic, blueview_topic,
                 blueview_frame_id, debug=True):

        self.enabled = False
        self.max_range = 10

        self.threshold = 0#0.0000 * np.iinfo(np.uint16).max

        rospy.init_node("image_blueview_fussion")
        super(ImageBlueviewFussion, self).__init__(left_image_topic, right_image_topic)

        self.bv_sub = rospy.Subscriber(blueview_topic, numpy_msg(BlueViewPing),
                                       self.bv_callback)
        self.debug_pub = Image_Publisher("~debug")

        self.wait_for_camera_model()

        # get tf from bv to left / right cam
        listener = tf.TransformListener()
        success = False
        while(not success):
            try:
                self.right_transform = listener.lookupTransform(
                                          self.camera_info_right.header.frame_id + '_optical',
                                          blueview_frame_id,
                                          rospy.Time(0))

                success = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        self.enabled = True

    def bv_callback(self, msg):
        if not self.enabled:
            return
        # get indicies of good entries
        good = np.argwhere(np.logical_and(msg.ranges < self.max_range,
                                          msg.intensities > self.threshold))
        if len(good) == 0:
            rospy.logerr('no good pings from the imaging sonar')
            return
        #TODO: threshold the blueview

        # overlay the bv data onto the cam images
        data_polar = np.array([msg.bearings[good],
                               msg.ranges[good]])
        data_polar = np.squeeze(data_polar)
        intensities = np.squeeze(msg.intensities[good])

        # translate these polar corrds to cartisian
        data_cartisian = np.apply_along_axis(lambda x:
                                    np.array([x[1] * np.cos(x[0]),
                                              x[1] * np.sin(x[0]),
                                              0.]),
                                    0, data_polar)

        # transform these points from the blueview frame to the right camera frame
        data_right = np.apply_along_axis(lambda x:
                       rotate_vect_by_quat(x[:3], self.right_transform[1]) +
                       self.right_transform[0],
                       0,
                       data_cartisian)
        try:
            pixels_right, depths = project_points3d_to_pixels_depths(self.camera_info_right,
                                                                     data_right,
                                                                     self.camera_model_right)
            pixel_depth_right = np.vstack((pixels_right, depths))
        except Exception as e:
            rospy.logerr(e)
            return

        np.apply_along_axis(lambda x: cv2.circle(self.last_image_right,
                              tuple(x[:2].astype(np.int16)), 1,
                              [0,0,int(x[2]*25)], -1),
                            0, pixel_depth_right)

        self.debug_pub.publish(self.last_image_right)



if __name__ == '__main__':
    a = ImageBlueviewFussion('/camera/front/left/image_rect_color',
                             '/camera/front/right/image_rect_color',
                             '/blueview_driver/ranges',
                             'blueview')
    rospy.spin()


