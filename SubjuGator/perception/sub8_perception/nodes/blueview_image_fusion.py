#!/usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np

from mil_blueview_driver.msg import BlueViewPing
from mil_ros_tools import ImageSubscriber, ImagePublisher


class ImageBlueviewFussion (StereoSubscriber):
    __init__(self, left_image_topic, right_image_topic, blueview_topic, debug=True):
        rospy.init_node("image_blueview_fussion")
        super(ImageBlueviewFussion, self).__init__(left_image_topic, right_image_topic)
        self.bv_sub = rospy.Subscriber(blueview_topic, numpy_msg(BlueViewPing),
                                       self.bv_callback, 1)
        self.threshold = 0.5
        self.normalizer = None
        self.debug_pub = ImagePublisher("~debug")


    bv_callback(self, msg):
        if self.normalizer is None:
            
        # threshold the blueview image
        np.where(msg.intensities
        # publish for debub if people are listening
        print 'a'


if __name__ == '__main__':
    a = ImageBlueViewFussion('/camera/front/left/image_rect_color',
                             '/camera/front/right/image_rect_color',
                             '/blueview_driver/ranges')
    rospy.spin()


