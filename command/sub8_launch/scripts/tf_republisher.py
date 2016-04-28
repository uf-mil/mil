#!/usr/bin/env python

import rospy
import tf
from uf_common.msg import Float64Stamped


def got_range(msg):
    '''TODO:
        - Make parallel to surface
    '''
    translation = (0.0, 0.0, -msg.data)
    if rospy.Time.now() < rospy.Time(0.5):
        listener.clear()
    t = rospy.Time(0)
    trans, rot = listener.lookupTransform("/base_link", "/map", t)

    bc.sendTransform(translation, rot, rospy.Time.now(), "/ground", "/dvl")


if __name__ == '__main__':
    rospy.init_node('tf_republish')
    rospy.sleep(1)
    bc = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rospy.sleep(1)

    sub = rospy.Subscriber('/dvl/range', Float64Stamped, got_range)

    rospy.spin()
